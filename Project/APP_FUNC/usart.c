#include "usart.h"
							 
//初始化IO 串口2
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart2_init(u32 pclk2,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
		temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
		mantissa=temp;				 //得到整数部分
		fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
			mantissa<<=4;
		mantissa+=fraction; 
		RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
		RCC->APB1ENR|=1<<17;  	//使能串口2时钟 
		GPIO_Set(GPIOA,GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP);//PA2,PA3,复用功能,上拉输出
		GPIO_AF_Set(GPIOA,2,7);	//PA2,AF7
		GPIO_AF_Set(GPIOA,3,7);//PA3,AF7  	   
		//波特率设置
		USART2->BRR=mantissa; 	//波特率设置	 
		USART2->CR1&=~(1<<15); 	//设置OVER8=0 
		USART2->CR1|=1<<3;  	//串口发送使能 
	#if EN_USART2_RX		  	//如果使能了接收
		//使能接收中断 
		USART2->CR1|=1<<2;  	//串口接收使能
		USART2->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(3,3,USART2_IRQn,2);//组2，最低优先级 
	#endif
		USART2->CR1|=1<<13;  	//串口使能
}


//初始化IO 串口6
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart6_init(u32 pclk2,u32 bound)
{  	 //PC6,PC7
		float temp;
		u16 mantissa;
		u16 fraction;	   
		temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
		mantissa=temp;				 //得到整数部分
		fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
			mantissa<<=4;
		mantissa+=fraction; 
		RCC->AHB1ENR|=1<<2;   	//使能PORTC口时钟
		RCC->APB2ENR|=1<<5;  	//使能串口6时钟 
		GPIO_Set(GPIOC,GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP);//PC6,PC7,复用功能,上拉输出
		GPIO_AF_Set(GPIOC,6,8);	//PC6,AF8
		GPIO_AF_Set(GPIOC,7,8);//PC7,AF8   
		//波特率设置
		USART6->BRR=mantissa; 	//波特率设置	 
		USART6->CR1&=~(1<<15); 	//设置OVER8=0 
		USART6->CR1|=1<<3;  	//串口发送使能 
		//使能接收中断 
//		USART6->CR1|=1<<2;  	//串口接收使能
//		USART6->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(3,3,USART6_IRQn,2);//组2，最低优先级 
		USART6->CR1|=1<<13;  	//串口使能
}



/****************************串口中断发送方式1*********************************************************/
////定义全局变量。也可以为了简化，把这四个参数结合起来包含在一个结构体里
//u8 TxLength;   //发送数据长度
//u8 TxIndicator;//发送指示器，表示目前发送完成哪一位了，下面要发送的是第（TxIndicator+1）位
//u8 TxBuff[256];//Data 
//u8 TxFnd;      //发送完成标志

//void Usart6_IRQ ( void )
//{
//	if (USART_GetITStatus(USART6, USART_IT_TC) != RESET)
//	{
//		USART_ClearITPendingBit(USART6,USART_IT_TC);
//		if( TxIndicator < TxLength  )//数组的索引max永远小于数组元素的个数
//		{
//			USART_SendData(USART6, TxBuff[TxIndicator++]);
//		}
//		else//最后一字节数据发送完成
//		{
//			TxFnd = 0;
//			TxIndicator = 0;
//		}
//	}
//}

//void Usart6_Send ( u8 *DataToSend , u8 data_num )
//{
//	u8 i;
//	TxLength=0;
//	for ( i = 0; i < data_num; i++ )
//	{
//			TxBuff[TxLength++] = * ( DataToSend + i );
//	}
//	TxIndicator = 1    ;//0已经发送，也是用来启动发送的
// 
//	USART_SendData(USART6, TxBuff[0]);  /**@Notes：只发送了txbuf的第一个字节*/
//}
/****************https://blog.csdn.net/qq_35629563/article/details/80879819*****************************/





/****************************************串口中断发送方式2**********************************************/
u8 FLAG_TC=0;//定义全局变量

void Usart6_IRQ ( void ){
 if (USART_GetITStatus(USART6, USART_IT_TC) != RESET)//发送完成中断,= SET
  {
    USART_ClearITPendingBit(USART6,USART_IT_TC);
    FLAG_TC=1;
  }
}

void Usart6_Send ( u8 *DataToSend , u8 data_num ){
	u8 i=0;
  FLAG_TC=0;//提前准备一下
  for(i=0;i<data_num;i++)//检测字符串结束符
  {
    USART_SendData(USART6 ,*(DataToSend+i));//发送当前字符
		if(!(USART6->CR1 & USART_CR1_TCIE))//判断USART6->CR1中的TCIE位是否设置，即“发送完成中断”是否使能
		{
			USART_ITConfig(USART6, USART_IT_TC, ENABLE);
		}
    while( FLAG_TC==0);	//0：发送还未完成；1：发送完成
    FLAG_TC=0;
  }
}
/*********************https://blog.csdn.net/qq_35629563/article/details/80879819***********************/
