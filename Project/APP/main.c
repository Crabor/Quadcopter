#include "includes.h"

//为了各函数调用全局变量方便以及如果把全局变量放到各子文件会出现种种bug,
//所以将大部分全局变量统一定义在主文件，其他子文件要用时用extern引用

/**********************************姿态解算相关******************************************************/
uint8_t gyroOffset = 0; //用于零偏校准
uint8_t accOffset = 0;
Acc acc, offsetAcc; //原始数据、零偏数据
Gyro gyro, offsetGyro; //原始数据、零偏数据
Mag mag;//原始数据
Float fGyro; //角速度数据（rad）
Angle angle; //姿态解算-角度值
// float ACC_IIR_FACTOR;
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
u16 PWM_IN_CH[4];//PWM输入通道带宽
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
// 函数定义
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

//static void Task_Startup(void* p_arg)
//{
//// etect OS task current capacity
//#if (OS_TASK_STAT_EN > 0)
//    OSStatInit();
//#endif
//    //最低占空比启动电机
//    TIM3->CCR1 = 54;
//    TIM3->CCR2 = 54;
//    TIM3->CCR3 = 54;
//    TIM3->CCR4 = 54;
//    OSTimeDly(5000);
//    Open_Calib(); //打开零偏校准

//    // Create functional task
//    OSTaskCreate(Task_Angel, (void*)0, &Task_Angel_STK[TASK_ANGEL_STK_SIZE - 1], TASK_ANGEL_PRIO);
//    OSTaskCreate(Task_PID, (void*)0, &Task_PID_STK[TASK_PID_STK_SIZE - 1], TASK_PID_PRIO);

//    // Delete itself
//    OSTaskDel(OS_PRIO_SELF);
//}

// 姿态解算任务
static void Task_Angel(void* p_arg)
{
    while (1) {
        MPU9150_Read();//读取九轴数据
        SendSenser(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z); //发送传感器原始数据帧
        if (!Calib_Status()) {//零偏校准结束
            IMUUpdate(fGyro.x, fGyro.y, fGyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z); //姿态解算
            SendAttitude(angle.roll, angle.pitch, angle.yaw);//发送姿态数据帧
        }
        OSTimeDly(1);
    }
}

// PID任务
static void Task_PID(void* p_arg)
{
    while (1) {
        if (!Calib_Status()) {//零偏校准结束
            Motor_Exp_Calc();// 计算遥控器的期望值
            Motor_Calc();// 计算PID以及要输出的电机速度
            PWM_OUT();// 输出电机速度
        }
        OSTimeDly(5);
    }
}

static void Task_Startup(void* p_arg)
{
    u16 temp;
    //最低占空比启动电机
    TIM3->CCR1 = 54;
    TIM3->CCR2 = 54;
    TIM3->CCR3 = 54;
    TIM3->CCR4 = 54;
    OSTimeDly(5000);
    Open_Calib();
    while (1) {
        OSTimeDly(1);
        MPU9150_Read();
        SendSenser(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z); 
        if (!Calib_Status()) {
            IMUUpdate(fGyro.x, fGyro.y, fGyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z);
            SendAttitude(angle.roll, -angle.pitch, -angle.yaw);
            TIM3->CCR1 = PWM_IN_CH[2] * PWM_IN_TO_OUT;
            TIM3->CCR2 = PWM_IN_CH[2] * PWM_IN_TO_OUT;
            TIM3->CCR3 = PWM_IN_CH[2] * PWM_IN_TO_OUT;
            TIM3->CCR4 = PWM_IN_CH[2] * PWM_IN_TO_OUT;
        }
    }
}
