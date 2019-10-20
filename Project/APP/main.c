#include "includes.h"

//为了方便各函数调用全局变量方便以及如果把全局变量放到各子文件会出现种种bug,
//所以将大部分全局变量统一定义在主文件，其他子文件要用时用extern引用

/**********************************姿态解算相关******************************************************/
uint8_t gyroOffset = 0; //不自动校正，用于零偏校准
uint8_t accOffset = 0;
Acc acc, filterAcc, offsetAcc; //原始数据、滤波后数据、零偏数据
Gyro gyro, filterGyro, offsetGyro; //原始数据、滤波后数据、零偏数据
Mag mag;
Float fAcc, fGyro, fMag; //加速度数据（m/s2）、角速度数据（rad）、磁场强度数据（Gs）
Angle angle; //姿态解算-角度值
float ACC_IIR_FACTOR;
/*******************************************************************************************************/

/***********************************PID相关*********************************************************/
PID rollCore, rollShell, pitchCore, pitchShell, yawCore, yawShell; //六个环的pid结构体
float pidT; //采样周期
int16_t pidRoll, pidPitch, pidYaw; //pid输出
float expRoll, expPitch, expYaw, expThr; //期望值
/*******************************************************************************************************/

/*************************************串口中断发送***********************************************************/
u8 sendBuf[50]; //发送数据缓存
/*******************************************************************************************************/

/***********************************PWM输入捕获******************************************************/
uint16_t PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4;
/*******************************************************************************************************/

/***********************************PWM输出比较*********************************************************/
int16_t motor1, motor2, motor3, motor4; //四个电机速度:左前顺时针，右前逆时针，左后逆时针，右后顺时针
/*******************************************************************************************************/

/**********************************操作系统相关*********************************************************/
// Tasks priority definition
#define TASK_STARTUP_PRIO 4
#define TASK_ANGEL_PRIO 5
#define TASK_PID_PRIO 6
// Tasks stack size definition
#define TASK_STARTUP_STK_SIZE 1024
#define TASK_ANGEL_STK_SIZE 512
#define TASK_PID_STK_SIZE 512
// Stack allocation
static OS_STK Task_Startup_STK[TASK_STARTUP_STK_SIZE];
static OS_STK Task_Angel_STK[TASK_ANGEL_STK_SIZE];
static OS_STK Task_PID_STK[TASK_PID_STK_SIZE];
// Functions definition
static void Task_Startup(void* p_arg);
static void Task_Angel(void* p_arg);
static void Task_PID(void* p_arg);
/*******************************************************************************************************/

int main(void)
{
	  BSP_Init();
    OSInit();
    OSTaskCreate(Task_Startup, (void*)0, &Task_Startup_STK[TASK_STARTUP_STK_SIZE - 1], TASK_STARTUP_PRIO);
    OSStart();
    return 0;
}

static void Task_Startup(void* p_arg)
{
// etect OS task current capacity
#if (OS_TASK_STAT_EN > 0)
    OSStatInit();
#endif
    //最低占空比启动电机
    TIM3->CCR1 = 54;
    TIM3->CCR2 = 54;
    TIM3->CCR3 = 54;
    TIM3->CCR4 = 54;
    OSTimeDly(5000);
    Open_Calib(); //打开零偏校准

    // Create functional task
    OSTaskCreate(Task_Angel, (void*)0, &Task_Angel_STK[TASK_ANGEL_STK_SIZE - 1], TASK_ANGEL_PRIO);
    OSTaskCreate(Task_PID, (void*)0, &Task_PID_STK[TASK_PID_STK_SIZE - 1], TASK_PID_PRIO);

    // Delete itself
    OSTaskDel(OS_PRIO_SELF);
}

// Function entry of AHRS task
static void Task_Angel(void* p_arg)
{
    while (1) {
        MPU9150_Read();
        SendSenser(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z); //发送传感器原始数据帧
        if (!Calib_Status()) {
            ACC_IIR_Filter(&acc, &filterAcc); //对acc做IIR滤波
            Gyro_Filter(&gyro, &filterGyro); //对gyro做窗口滤波
            Get_Radian(&filterGyro, &fGyro); //角速度数据转为弧度
            IMUUpdate(fGyro.x, fGyro.y, fGyro.z, filterAcc.x, filterAcc.y, filterAcc.z, mag.x, mag.y, mag.z); //姿态解算
            Get_Eulerian_Angle(&angle);
            SendAttitude(angle.roll, angle.pitch, angle.yaw);
        }
        OSTimeDly(1);
    }
}

// Function entry of PID control task
static void Task_PID(void* p_arg)
{
    while (1) {
				if (!Calib_Status()){
						// Remote control value processing
						Motor_Exp_Calculate(PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4);
						// Motor PID calculation
						Motor_Calculate();
						// Output PWM to motors
						PWM_OUT();
				}
        // OS time delay 5 ticks
        OSTimeDly(5);
    }
}

//static void Task_Startup(void* p_arg)
//{
//		 u16 temp;
//     //最低占空比启动电机
//     TIM3->CCR1 = 54;
//     TIM3->CCR2 = 54;
//     TIM3->CCR3 = 54;
//     TIM3->CCR4 = 54;
//     OSTimeDly(5000);
//		 temp=60;
//	   TIM3->CCR1=temp;//PWM输出
//     TIM3->CCR2=temp;//PWM输出
//     TIM3->CCR3=temp;//PWM输出
//     TIM3->CCR4=temp;//PWM输出
//     while (1);
//}
