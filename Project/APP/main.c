#include "includes.h"

/**********************************姿态解算相关数据******************************************************/
uint8_t gyroOffset = 0; //不自动校正，用于零偏校准
uint8_t accOffset = 0;
Acc acc, filterAcc, offsetAcc; //原始数据、滤波后数据、零偏数据
Gyro gyro, filterGyro, offsetGyro; //原始数据、滤波后数据、零偏数据
Mag mag;
Float fAcc, fGyro, fMag; //加速度数据（m/s2）、角速度数据（rad）、磁场强度数据（Gs）
Angle angle; //姿态解算-角度值
PID pitch, roll, gyroPitch, gyroRoll, gyroYaw;
float ACC_IIR_FACTOR;
/*******************************************************************************************************/

/*************************************串口中断发送***********************************************************/
u8 testdatatosend[50]; //发送数据缓存
/*******************************************************************************************************/

/***********************************PWM输入捕获******************************************************/
uint16_t PWMInCh1 = 0, PWMInCh2 = 0, PWMInCh3 = 0, PWMInCh4 = 0;
/*******************************************************************************************************/

/***********************************PWM输出比较*********************************************************/

/*******************************************************************************************************/

/**********************************操作系统相关*********************************************************/
#define TASK_1_PRIO 3
#define TASK_1_STK_SIZE 2048

static OS_STK Task_1_STK[TASK_1_STK_SIZE];
static void Task_1(void* p_arg);
/*******************************************************************************************************/

int main(void)
{
    BSP_Init();
    OSInit();
    OSTaskCreate(Task_1, (void*)0, &Task_1_STK[TASK_1_STK_SIZE - 1], TASK_1_PRIO);
    OSStart();
    return 0;
}

static void Task_1(void *p_arg){
	//最低占空比启动电机
	TIM3->CCR1 = 54;
	TIM3->CCR2 = 54;
	TIM3->CCR3 = 54;
	TIM3->CCR4 = 54;
	OSTimeDly(5000);
	while(1){
//		OSTimeDly(1000);
//		SendStr("nihao");
//			TIM3->CCR1=PWMInCh3*0.054;//PWM输出
//			TIM3->CCR2=PWMInCh3*0.054;//PWM输出
//			TIM3->CCR3=PWMInCh3*0.054;//PWM输出
//			TIM3->CCR4=PWMInCh3*0.054;//PWM输出
	}
}


//static void Task_1(void* p_arg)
//{
//    Open_Calib(); //打开零偏校准
//    while (1) {
//        OSTimeDly(1);
//        MPU9150_Read();
////        SendSenser(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z); //发送传感器原始数据帧
//        if (!Calib_Status()) {
//            ACC_IIR_Filter(&acc, &filterAcc); //对acc做IIR滤波
//            Gyro_Filter(&gyro, &filterGyro); //对gyro做窗口滤波
//            Get_Radian(&filterGyro, &fGyro); //角速度数据转为弧度
//            IMUUpdate(fGyro.x, fGyro.y, fGyro.z, filterAcc.x, filterAcc.y, filterAcc.z, mag.x, mag.y, mag.z); //姿态解算
//            Get_Eulerian_Angle(&angle);
//            SendAttitude(angle.roll, angle.pitch, angle.yaw);
//        }
//    }
//}
