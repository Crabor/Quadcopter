#include "includes.h"

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

extern u8 TIM5CH1_CAPTURE_STA; //输入捕获状态
extern u32 TIM5CH1_CAPTURE_VAL; //输入捕获值


#define TASK_1_PRIO 3

#define TASK_1_STK_SIZE 2048

static OS_STK Task_1_STK[TASK_1_STK_SIZE];

static void Task_1(void *p_arg);

int main(void){
	BSP_Init();
	OSInit();
	OSTaskCreate(Task_1, (void *)0, &Task_1_STK[TASK_1_STK_SIZE - 1], TASK_1_PRIO);
	OSStart();
	return 0;
} 


static void Task_1(void *p_arg){
  while(1){
	  printf("Hello STM32!\n");
	  OSTimeDly(1000);
  }
}

//static void Task_1(void *p_arg){
//	uint32_t lowPulse=54;
//	long long temp=0;
//	//最低占空比启动电机
//	TIM3->CCR1 = lowPulse;
//	TIM3->CCR2 = lowPulse;
//	TIM3->CCR3 = lowPulse;
//	TIM3->CCR4 = lowPulse;
//	OSTimeDly(5000);
//	printf("Open quadcopter successful!\r\n");
//	OSTimeDly(1000);
//	MPU6050_Init();
//	while(1)
//	{
//		OSTimeDly(100);
//		if(TIM5CH1_CAPTURE_STA&0X80) //成功捕获到了一次高电平
//		{
//			temp=TIM5CH1_CAPTURE_STA&0X3F;
//			temp*=0XFFFFFFFF; //溢出时间总和
//			temp+=TIM5CH1_CAPTURE_VAL; //得到总的高电平时间
//			printf("Duty:%f %%\r\n",temp*1.0/18600); //打印占空比
//			TIM3->CCR1 = temp/18.6;
//			TIM3->CCR2 = temp/18.6;
//			TIM3->CCR3 = temp/18.6;
//			TIM3->CCR4 = temp/18.6;
//			TIM5CH1_CAPTURE_STA=0; //开启下一次捕获
//		}
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

