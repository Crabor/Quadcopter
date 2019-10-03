#include "includes.h"

/**********************************姿态解算相关数据******************************************************/
uint8_t	gyroOffset = 0;//不自动校正，用于零偏校准
uint8_t	accOffset  = 0;
Acc acc,filterAcc,offsetAcc;//原始数据、滤波后数据、零偏数据
Gyro gyro,filterGyro,offsetGyro;//原始数据、滤波后数据、零偏数据
Mag mag;
Float fAcc,fGyro;//加速度数据（m/s2）、角速度数据（rad）
Angle angle;//姿态解算-角度值
PID pitch,roll,gyroPitch,gyroRoll,gyroYaw;
float ACC_IIR_FACTOR;
/*******************************************************************************************************/

/*************************************串口中断发送***********************************************************/
u8 testdatatosend[50];	//发送数据缓存
/*******************************************************************************************************/

/***********************************PWM输入捕获******************************************************/
uint16_t PWMInCh1=0, PWMInCh2=0, PWMInCh3=0, PWMInCh4=0;
/*******************************************************************************************************/

/***********************************PWM输出比较*********************************************************/

/*******************************************************************************************************/

/**********************************操作系统相关*********************************************************/
#define TASK_1_PRIO 3
#define TASK_1_STK_SIZE 2048

static OS_STK Task_1_STK[TASK_1_STK_SIZE];
static void Task_1(void *p_arg);
/*******************************************************************************************************/

int main(void){
	BSP_Init();
	OSInit();
	OSTaskCreate(Task_1, (void *)0, &Task_1_STK[TASK_1_STK_SIZE - 1], TASK_1_PRIO);
	OSStart();
	return 0;
} 

//static void Task_1(void *p_arg){
//	u32 cnt=0;
//	//最低占空比启动电机
////	TIM3->CCR1 = 54;
////	TIM3->CCR2 = 54;
////	TIM3->CCR3 = 54;
////	TIM3->CCR4 = 54;
////	OSTimeDly(5000);
//	while(1){
////		ANO_DT_SendString("1");
//		OSTimeDly(1000);
////			TIM3->CCR1=PWMInCh3*0.054;//PWM输出
////			TIM3->CCR2=PWMInCh3*0.054;//PWM输出
////			TIM3->CCR3=PWMInCh3*0.054;//PWM输出
////			TIM3->CCR4=PWMInCh3*0.054;//PWM输出
//		cnt++;
//		SendWord(&cnt);
////		SendHalfWord(&PWMInCh3);
//	}
//}


//static void Task_1(void *p_arg){
//	u16 lowPulse=135;//5.4%*2500
//	u16 temp;
//	lowPulse=54;
//	//最低占空比启动电机
//	TIM3->CCR1 = lowPulse;
//	TIM3->CCR2 = lowPulse;
//	TIM3->CCR3 = lowPulse;
//	TIM3->CCR4 = lowPulse;
//	OSTimeDly(5000);
////	SendString("Quadcopter open successfully!\n");
////	OSTimeDly(1000);
//	// while(MPU6050_Init()!=1);//若MPU6050初始化不成功，则程序不向下运行;因为调用了OSTimeDly()函数，所以不在BSP.c里初始化
//	while(1)
//	{
////		SendString("In the while loop\r\n");
//		
//		OSTimeDly(100);
//		temp=PWM_IN_CH[0]*PWM_IN_TO_OUT;
//		TIM3->CCR1 = temp;
//		TIM3->CCR2 = temp;
//		TIM3->CCR3 = temp;
//		TIM3->CCR4 = temp;
//	}
//}


 static void Task_1(void *p_arg){
//	 u32 cnt=100;
 //	int16_t ACCEL_X=0,ACCEL_Y=0,ACCEL_Z=0,GYRO_X=0,GYRO_Y=0,GYRO_Z=0;
//	ANO_DT_SendString("1");
 //  while(1){
 //	  printf("Hello STM32!\n");
 //	  OSTimeDly(1000);
 //  }
	 
 	Open_Calib();//打开零偏校准
 //	while(1){
 //		OSTimeDly(1);
 //		MPU6050_Read();
 //		if(accOffset==0 && gyroOffset==0) break;
 //	}
 	 while(1){
 	   OSTimeDly(10);
//		 cnt++;
//	   SendWord(&cnt);
 		MPU6050_Read();
//		ANO_DT_SendString("2");
//		SendSenser(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,mag.x,mag.y,mag.z);//发送传感器原始数据帧
		 SendSenser(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,mag.x,mag.y,mag.z);//发送传感器原始数据帧
// 		ACC_IIR_Filter(&acc,&filterAcc);//对acc做IIR滤波
// 		Gyro_Filter(&gyro,&filterGyro);//对gyro做窗口滤波
// 		Get_Radian(&filterGyro,&fGyro);//角速度数据转为弧度
// 		IMUupdate(fGyro.x,fGyro.y,fGyro.z,filterAcc.x,filterAcc.y,filterAcc.z,mag.x,mag.y,mag.z);//姿态解算
 //		IMUupdate(gyro.x,gyro.y,gyro.z,acc.x,acc.y,acc.z);
// 		Get_Eulerian_Angle(&angle);
// 		SendAttitude(angle.roll,angle.pitch,angle.yaw);
		
  	}
	 
 }
 
// void SendSenser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z,int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z,int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z)	//发送用户数据，这里有6个数据
//{
////	u8 _cnt=0;
////	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
////	int i;
////	ACCEL_X=1;
////	ACCEL_Y=2;
////	ACCEL_Z=3;
////	GYRO_X=4;
////	GYRO_Y=5;
////	GYRO_Z=6;
////	MAG_X=7;
////	MAG_Y=8;
////	MAG_Z=9;
////	
////	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
////	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
////	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
////	testdatatosend[_cnt++]=0xF1;//0x02，表示本帧为传感器原始数据帧
////	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
//// 
////	testdatatosend[_cnt++]=BYTE1(ACCEL_X);//将要发送的数据放至发送缓冲区
////	testdatatosend[_cnt++]=BYTE0(ACCEL_X);
////	
////	testdatatosend[_cnt++]=BYTE1(ACCEL_Y);
////	testdatatosend[_cnt++]=BYTE0(ACCEL_Y);
////	
////	testdatatosend[_cnt++]=BYTE1(ACCEL_Z);
////	testdatatosend[_cnt++]=BYTE0(ACCEL_Z);
////	
////	testdatatosend[_cnt++]=BYTE1(GYRO_X);
////	testdatatosend[_cnt++]=BYTE0(GYRO_X);
////	
////	testdatatosend[_cnt++]=BYTE1(GYRO_Y);
////	testdatatosend[_cnt++]=BYTE0(GYRO_Y);
////	
////	testdatatosend[_cnt++]=BYTE1(GYRO_Z);
////	testdatatosend[_cnt++]=BYTE0(GYRO_Z);
////	
////	testdatatosend[_cnt++]=BYTE1(MAG_X);
////	testdatatosend[_cnt++]=BYTE0(MAG_X);
////	
////	testdatatosend[_cnt++]=BYTE1(MAG_Y);
////	testdatatosend[_cnt++]=BYTE0(MAG_Y);
////	
////	testdatatosend[_cnt++]=BYTE1(MAG_Z);
////	testdatatosend[_cnt++]=BYTE0(MAG_Z);
////	
////	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
////	
////	for(i=0;i<_cnt;i++)
////		sum += testdatatosend[i];
////	
////	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
////	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
//	u8 _cnt=0;
//	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
//	int i;
//	
//	ACCEL_X=1;
//	ACCEL_Y=2;
//	ACCEL_Z=3;
//	GYRO_X=4;
//	GYRO_Y=5;
//	GYRO_Z=6;
//	MAG_X=7;
//	MAG_Y=8;
//	MAG_Z=9;
//	
//	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
//	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
//	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
//	testdatatosend[_cnt++]=0xF1;//0x02，表示本帧为传感器原始数据帧
//	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
// 
//	testdatatosend[_cnt++]=BYTE1(ACCEL_X);//将要发送的数据放至发送缓冲区
//	testdatatosend[_cnt++]=BYTE0(ACCEL_X);
//	
//	testdatatosend[_cnt++]=BYTE1(ACCEL_Y);
//	testdatatosend[_cnt++]=BYTE0(ACCEL_Y);
//	
//	testdatatosend[_cnt++]=BYTE1(ACCEL_Z);
//	testdatatosend[_cnt++]=BYTE0(ACCEL_Z);
//	
//	testdatatosend[_cnt++]=BYTE1(GYRO_X);
//	testdatatosend[_cnt++]=BYTE0(GYRO_X);
//	
//	testdatatosend[_cnt++]=BYTE1(GYRO_Y);
//	testdatatosend[_cnt++]=BYTE0(GYRO_Y);
//	
//	testdatatosend[_cnt++]=BYTE1(GYRO_Z);
//	testdatatosend[_cnt++]=BYTE0(GYRO_Z);
//	
//	testdatatosend[_cnt++]=BYTE1(MAG_X);
//	testdatatosend[_cnt++]=BYTE0(MAG_X);
//	
//	testdatatosend[_cnt++]=BYTE1(MAG_Y);
//	testdatatosend[_cnt++]=BYTE0(MAG_Y);
//	
//	testdatatosend[_cnt++]=BYTE1(MAG_Z);
//	testdatatosend[_cnt++]=BYTE0(MAG_Z);
//	
//	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
//	
//	for(i=0;i<_cnt;i++)
//		sum += testdatatosend[i];
//	
//	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
//	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数

//}


