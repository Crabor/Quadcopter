#include "includes.h"
#include "math.h"
//为了各函数调用全局变量方便以及如果把全局变量放到各子文件会出现种种bug,
//所以将大部分全局变量统一定义在主文件，其他子文件要用时用extern引用

/**********************************姿态解算相关******************************************************/
uint8_t gyroOffset, accOffset, pressOffset; //用于零偏校准
Acc acc, offsetAcc; //原始数据、零偏数据
Gyro gyro, offsetGyro; //原始数据、零偏数据
Mag mag; //原始数据
Float fGyro; //角速度数据（rad）
Angle angle; //姿态解算-角度值
float press, offsetPress; //温度补偿大气压，零偏大气压
float Temperature; //实际温度
float K_PRESS_TO_HIGH; //气压转换成高度，因为不同地区比例不一样，所以不设成宏
float height;//高度
/*******************************************************************************************************/

/***********************************PID相关*********************************************************/
PID rollCore, rollShell, pitchCore, pitchShell, yawCore; //五个环的pid结构体，yaw不需要外环
float pidT; //采样周期
float pidRoll, pidPitch, pidYaw; //pid输出
float expRoll, expPitch, expYaw, expThr; //期望值
/*******************************************************************************************************/

/*************************************串口中断发送***********************************************************/
//OS_EVENT *sendBufMutex;//发送数据缓存锁
u8 sendBuf[50]; //发送数据缓存
/*******************************************************************************************************/

/***********************************PWM输入捕获******************************************************/
u16 PWM_IN_CH[4]; //PWM输入通道带宽
/*******************************************************************************************************/

/***********************************PWM输出比较*********************************************************/
float motor1, motor2, motor3, motor4; //四个电机速度:左前顺时针，右前逆时针，左后逆时针，右后顺时针
/*******************************************************************************************************/

/**********************************操作系统相关*********************************************************/
// 任务优先级定义
#define TASK_STARTUP_PRIO 4
#define TASK_COM_PRIO 8
#define TASK_ANGEL_PRIO 6
#define TASK_PID_PRIO 7
// 任务栈大小定义
#define TASK_STARTUP_STK_SIZE 1024
#define TASK_COM_STK_SIZE 512
#define TASK_ANGEL_STK_SIZE 512
#define TASK_PID_STK_SIZE 512
// 栈内存分配
static OS_STK Task_Startup_STK[TASK_STARTUP_STK_SIZE];
static OS_STK Task_COM_STK[TASK_COM_STK_SIZE];
static OS_STK Task_Angel_STK[TASK_ANGEL_STK_SIZE];
static OS_STK Task_PID_STK[TASK_PID_STK_SIZE];
// 函数定义
static void Task_Startup(void* p_arg);
static void Task_COM(void* p_arg);
static void Task_Angel(void* p_arg);
static void Task_PID(void* p_arg);
/*******************************************************************************************************/

int main(void)
{
    OSInit();
    OSTaskCreate(Task_Startup, (void*)0, &Task_Startup_STK[TASK_STARTUP_STK_SIZE - 1], TASK_STARTUP_PRIO);
    OSStart();
    return 0;
}

// static void Task_Startup(void* p_arg)
// {
//     //INT8U err;
// // etect OS task current capacity
// #if (OS_TASK_STAT_EN > 0)
//     OSStatInit();
// #endif
//     //最低占空比启动电机
//     TIM3->CCR1 = 54;
//     TIM3->CCR2 = 54;
//     TIM3->CCR3 = 54;
//     TIM3->CCR4 = 54;
//     OSTimeDly(5000);
//     Open_Calib(); //打开零偏校准

//     //Create mutex
//     //sendBufMutex=OSMutexCreate(5,&err);

//     // Create functional task
//     OSTaskCreate(Task_COM, (void*)0, &Task_COM_STK[TASK_COM_STK_SIZE - 1], TASK_COM_PRIO);
//     OSTaskCreate(Task_Angel, (void*)0, &Task_Angel_STK[TASK_ANGEL_STK_SIZE - 1], TASK_ANGEL_PRIO);
//     OSTaskCreate(Task_PID, (void*)0, &Task_PID_STK[TASK_PID_STK_SIZE - 1], TASK_PID_PRIO);

//     // Delete itself
//     OSTaskDel(OS_PRIO_SELF);
// }

static void Task_COM(void* p_arg)
{
    float P, I, D;
    while (1) {
        Send_Senser(acc.x, acc.y, acc.z, gyro.x, gyro.y * RAW_TO_ANGLE, gyro.z, mag.x, mag.y, mag.z); //发送传感器原始数据帧
        Send_RCData_Motor(PWM_IN_CH[2], PWM_IN_CH[0], PWM_IN_CH[3], PWM_IN_CH[1], motor1, motor2, motor3, motor4); //发送遥控器数据和电机速度数据帧
        Send_expVal(0xF1, expRoll, expPitch, expYaw, expThr); //发送遥控器数据转换成的期望值
        if (!Calib_Status()) { //零偏校准结束
            Send_Attitude(angle.roll, angle.pitch, angle.yaw); //发送姿态数据帧
            P = rollCore.Kp * rollCore.eK;
            I = rollCore.Kp * rollCore.eSum * pidT / rollCore.Ti;
            D = rollCore.Kp * (rollCore.eK - rollCore.eK_1) * rollCore.Td / pidT;
            Send_pidOutVal(0xF2, rollShell.output, rollCore.output, P, I, D, yawCore.output); //发送内外环PID计算结果
        }
        OSTimeDly(10);
    }
}

// 姿态解算任务
static void Task_Angel(void* p_arg)
{
    //INT8U err;
    while (1) {
        GY86_Read(); //读取九轴数据
        if (!Calib_Status()) { //零偏校准结束
            AttitudeUpdate(fGyro.x, fGyro.y, fGyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z); //姿态解算
        }
        OSTimeDly(1);
    }
}

// PID任务
static void Task_PID(void* p_arg)
{
    //INT8U err;
    while (1) {
        Motor_Exp_Calc(); // 计算遥控器的期望值
        if (!Calib_Status()) { //零偏校准结束
            Motor_Calc(); // 计算PID以及要输出的电机速度
            PWM_OUT(); // 输出电机速度
        }
        OSTimeDly(3);
    }
}

static void Task_Startup(void* p_arg)
{
    int32_t t, h, p;
    BSP_Init();
    // etect OS task current capacity
#if (OS_TASK_STAT_EN > 0)
    OSStatInit();
#endif
    Open_Calib();
    while (1) {
        OSTimeDly(1);
        GY86_Read();
        if(!Calib_Status()){
            //t = Temperature;
            //h = 44300*(1-pow(press/101325,1/5.256))*100;
            p = press;
            h = press*K_PRESS_TO_HIGH*100;
            SendWord(0xF1, &p);
            SendWord(0xF2, &h);
            Send_Senser(acc.x, acc.y, acc.z, gyro.x, gyro.y * RAW_TO_ANGLE, gyro.z, mag.x, mag.y, mag.z); //发送传感器原始数据帧
        }

    }
}
