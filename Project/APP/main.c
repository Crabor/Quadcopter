#include "includes.h"

/************************匿名四轴上位机高级收码*********************************************************/
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
u8 testdatatosend[50];	//发送数据缓存
/*******************************************************************************************************/

extern u8 TIM5CH1_CAPTURE_STA; //输入捕获状态
extern u32 TIM5CH1_CAPTURE_VAL; //输入捕获值


#define TASK_1_PRIO 3

#define TASK_1_STK_SIZE 2048

static OS_STK Task_1_STK[TASK_1_STK_SIZE];

static void Task_1(void *p_arg);
void SendSenser(u16 ACCEL_X, u16 ACCEL_Y, u16 ACCEL_Z,u16 GYRO_X, u16 GYRO_Y, u16 GYRO_Z);

int main(void){
	BSP_Init();
	OSInit();
	OSTaskCreate(Task_1, (void *)0, &Task_1_STK[TASK_1_STK_SIZE - 1], TASK_1_PRIO);
	OSStart();
	return 0;
} 


static void Task_1(void *p_arg){
	int16_t ACCEL_X=0,ACCEL_Y=0,ACCEL_Z=0,GYRO_X=0,GYRO_Y=0,GYRO_Z=0;
//  while(1){
//	  printf("Hello STM32!\n");
//	  OSTimeDly(1000);
//  }
	
	while(1){
		OSTimeDly(10);
		
//		ACCEL_X=GetData_MPU6050(ACCEL_XOUT_H) / 16384.0;
//		ACCEL_Y=GetData_MPU6050(ACCEL_YOUT_H) / 16384.0;
//		ACCEL_Z=GetData_MPU6050(ACCEL_ZOUT_H) / 16384.0;
//		GYRO_X=GetData_MPU6050(GYRO_XOUT_H)*0.001064;
//		GYRO_Y=GetData_MPU6050(GYRO_YOUT_H)*0.001064;
//		GYRO_Z=GetData_MPU6050(GYRO_ZOUT_H)*0.001064;
		
		ACCEL_X=GetData_MPU6050(ACCEL_XOUT_H);
		ACCEL_Y=GetData_MPU6050(ACCEL_YOUT_H);
		ACCEL_Z=GetData_MPU6050(ACCEL_ZOUT_H);
		GYRO_X=GetData_MPU6050(GYRO_XOUT_H);
		GYRO_Y=GetData_MPU6050(GYRO_YOUT_H);
		GYRO_Z=GetData_MPU6050(GYRO_ZOUT_H);

		SendSenser(ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z);
	}
	
}

void SendSenser(u16 ACCEL_X, u16 ACCEL_Y, u16 ACCEL_Z,u16 GYRO_X, u16 GYRO_Y, u16 GYRO_Z)	//发送用户数据，这里有6个数据
{
	u8 _cnt=0;
	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	int i;
	int16_t MAG_X=0,MAG_Y=0,MAG_Z=0;
	
	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
	testdatatosend[_cnt++]=0x05;//0x00为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	testdatatosend[_cnt++]=0x02;//0xF1，表示本帧为F1用户自定义帧，对应高级收码的F1功能帧
	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
 
	testdatatosend[_cnt++]=BYTE1(ACCEL_X);//将要发送的数据放至发送缓冲区
	testdatatosend[_cnt++]=BYTE0(ACCEL_X);
	
	testdatatosend[_cnt++]=BYTE1(ACCEL_Y);
	testdatatosend[_cnt++]=BYTE0(ACCEL_Y);
	
	testdatatosend[_cnt++]=BYTE1(ACCEL_Z);
	testdatatosend[_cnt++]=BYTE0(ACCEL_Z);
	
	testdatatosend[_cnt++]=BYTE1(GYRO_X);
	testdatatosend[_cnt++]=BYTE0(GYRO_X);
	
	testdatatosend[_cnt++]=BYTE1(GYRO_Y);
	testdatatosend[_cnt++]=BYTE0(GYRO_Y);
	
	testdatatosend[_cnt++]=BYTE1(GYRO_Z);
	testdatatosend[_cnt++]=BYTE0(GYRO_Z);
	
	testdatatosend[_cnt++]=BYTE1(MAG_X);//假数据，为了符合数据帧要求
	testdatatosend[_cnt++]=BYTE0(MAG_X);
	
	testdatatosend[_cnt++]=BYTE1(MAG_Y);
	testdatatosend[_cnt++]=BYTE0(MAG_Y);
	
	testdatatosend[_cnt++]=BYTE1(MAG_Z);
	testdatatosend[_cnt++]=BYTE0(MAG_Z);
	
	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
 
}



//static void Task_1(void *p_arg){
////	uint32_t lowPulse=54;
//	long long temp=0;
////	//最低占空比启动电机
////	TIM3->CCR1 = lowPulse;
////	TIM3->CCR2 = lowPulse;
////	TIM3->CCR3 = lowPulse;
////	TIM3->CCR4 = lowPulse;
////	OSTimeDly(5000);
////	printf("Open quadcopter successful!\r\n");
////	OSTimeDly(1000);
//	MPU6050_Init();
//	while(1)
//	{
//		OSTimeDly(100);
////		if(TIM5CH1_CAPTURE_STA&0X80) //成功捕获到了一次高电平
////		{
////			temp=TIM5CH1_CAPTURE_STA&0X3F;
////			temp*=0XFFFFFFFF; //溢出时间总和
////			temp+=TIM5CH1_CAPTURE_VAL; //得到总的高电平时间
////			//printf("Duty:%f %%\r\n",temp*1.0/18600); //打印占空比
////			TIM3->CCR1 = temp/18.6;
////			TIM3->CCR2 = temp/18.6;
////			TIM3->CCR3 = temp/18.6;
////			TIM3->CCR4 = temp/18.6;
////			TIM5CH1_CAPTURE_STA=0; //开启下一次捕获
////		}
//		
//		/* 打印 x, y, z 轴加速度 */
//		printf("ACCEL_X: %lf\r\n", GetData_MPU6050(ACCEL_XOUT_H) / 16384.0);
//		printf("ACCEL_Y: %lf\r\n", GetData_MPU6050(ACCEL_YOUT_H) / 16384.0);
//		printf("ACCEL_Z: %lf\r\n", GetData_MPU6050(ACCEL_ZOUT_H) / 16384.0);
//		
//		/* 打印温度，需要根据手册的公式换算为摄氏度 */
//		printf("TEMP: %0.2f\r\n", GetData_MPU6050(TEMP_OUT_H) / 340.0 + 36.53);
//		
//		/* 打印 x, y, z 轴角速度 */
//		printf("GYRO_X: %lf\r\n", GetData_MPU6050(GYRO_XOUT_H)*0.001064);
//		printf("GYRO_Y: %lf\r\n", GetData_MPU6050(GYRO_YOUT_H)*0.001064);
//		printf("GYRO_Z: %lf\r\n", GetData_MPU6050(GYRO_ZOUT_H)*0.001064);
//		
//		printf("\r\n");
//	}
//}


