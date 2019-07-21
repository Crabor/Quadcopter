#include "includes.h"

//u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
////接收状态
////bit15，	接收完成标志
////bit14，	接收到0x0d
////bit13~0，	接收到的有效字节数目
//u16 USART_RX_STA=0;       //接收状态标记	


#define TASK_1_PRIO 4
#define TASK_2_PRIO 5

#define TASK_1_STK_SIZE 2048
#define TASK_2_STK_SIZE 2048

static OS_STK Task_1_STK[TASK_1_STK_SIZE];
static OS_STK Task_2_STK[TASK_2_STK_SIZE];

static void Task_1(void *p_arg);
static void Task_2(void *p_arg);
//static void Task_usart(void *p_arg);

int main(void){
	BSP_Init();
	OSInit();
	OSTaskCreate(Task_1, (void *)0, &Task_1_STK[TASK_1_STK_SIZE - 1], TASK_1_PRIO);
	//OSTaskCreate(Task_2, (void *)0, &Task_2_STK[TASK_2_STK_SIZE - 1], TASK_2_PRIO);
	OSStart();
	return 0;
} 

//static void Task_usart(void *p_arg){
////	u8 t;
////	u8 len;
////	u16 times=0;
////	while(1){
////		if(USART_RX_STA&0x8000)
////		{
////			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
////			printf("\r\nYou:\r\n");
////			for(t=0;t<len;t++)
////			{
////				USART1->DR=USART_RX_BUF[t];
////				while((USART1->SR&0X40)==0);//等待发送结束
////			}
////			printf("\r\n\r\n");//插入换行
////			USART_RX_STA=0;
////		}else
////		{
////			times++;
////			if(times%5000==0)
////			{
////				printf("\r\nALIENTEK STM32F407\r\n");
////			}
////			if(times%200==0)printf("Enter data, end with enter:\r\n");
////			OSTimeDly(10);
////		}
////	}
//	int i=1;
//	while(i++){
//		printf("%d\r\n",i);
//		OSTimeDly(1000);
//	}
//}


static void Task_1(void *p_arg){
	uint32_t lowPulse=55,highPulse=90;
	//最低占空比启动电机
	//TIM_SetCompare1(TIM3,lowPulse);
	TIM3->CCR1 = lowPulse;
	TIM3->CCR2 = lowPulse;
	TIM3->CCR3 = lowPulse;
	TIM3->CCR4 = lowPulse;
	//TIM3->CR1|=1<<0; //使能定时器 3
	printf("lowPulse/arr: %f\r\n",(float)TIM3->CCR1/(TIM3->ARR+1));
	OSTimeDly(5000);
	//TIM_SetCompare1(TIM3,highPulse);
	TIM3->CCR1 = highPulse;
	TIM3->CCR2 = highPulse;
	TIM3->CCR3 = highPulse;
	TIM3->CCR4 = highPulse;
	printf("highPulse/arr: %f\r\n",(float)TIM3->CCR1/(TIM3->ARR+1));
	while(1){
		printf("pulse: %f\r\n",(float)TIM3->CCR1/(TIM3->ARR+1));
		//TIM_SetCompare1(TIM3,highPulse);
		OSTimeDly(1000);
	}
}

static void Task_2(void *p_arg){
	int i=0,dir=1;
	while(1){
		if(dir)i++;
		else i--;
		if(i>=499){
			dir=0;
		}else if(i<=1){
			dir=1;
		}
		TIM2->CCR1=i;
		OSTimeDly(10);
	}
}
