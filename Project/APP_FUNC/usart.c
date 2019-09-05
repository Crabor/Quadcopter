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

void uart6_init(u32 bound){
	  USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_USART6, ENABLE ); //开启USART6时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOC, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource6, GPIO_AF_USART6 );
    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource7, GPIO_AF_USART6 );

    //配置PC6作为USART6　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );
    //配置PC7作为USART6　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );

    //配置USART6
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = bound;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx;  //发送使能
    //配置USART6时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART6, &USART_InitStructure );
    USART_ClockInit ( USART6, &USART_ClockInitStruct );

    //使能USART6接收中断
//    USART_ITConfig ( USART6, USART_IT_RXNE, ENABLE );
    //使能USART6
    USART_Cmd ( USART6, ENABLE );
//	//使能发送（进入移位）中断
//	if(!(USART6->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
//	}


}


//初始化IO 串口6
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
//void uart6_init(u32 pclk2,u32 bound)
//{  	 //PC6,PC7
//		float temp;
//		u16 mantissa;
//		u16 fraction;	   
//		temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
//		mantissa=temp;				 //得到整数部分
//		fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
//			mantissa<<=4;
//		mantissa+=fraction; 
//		RCC->AHB1ENR|=1<<2;   	//使能PORTC口时钟
//		RCC->APB2ENR|=1<<5;  	//使能串口6时钟 
//		GPIO_Set(GPIOC,GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP);//PC6,PC7,复用功能,上拉输出
//		GPIO_AF_Set(GPIOC,6,8);	//PC6,AF8
//		GPIO_AF_Set(GPIOC,7,8);//PC7,AF8   
//		//波特率设置
//		USART6->BRR=mantissa; 	//波特率设置	 
//		USART6->CR1&=~(1<<15); 	//设置OVER8=0 
//		USART6->CR1|=1<<3;  	//串口发送使能 
//		//使能接收中断 
////		USART6->CR1|=1<<2;  	//串口接收使能
////		USART6->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
//		MY_NVIC_Init(3,3,USART6_IRQn,2);//组2，最低优先级 
//		USART6->CR1|=1<<13;  	//串口使能
//}

u8 TxBuffer[256];
u8 count = 0;

void Usart6_IRQ ( void )
{
    //发送中断
    if ( USART_GetITStatus ( USART6, USART_IT_TXE ) )
    {
        usart6_sendstring(TxBuffer);
        USART_ITConfig ( USART6, USART_IT_TXE, DISABLE );		//关闭TXE（发送中断）中断
			  //USART_ClearITPendingBit(USART6,USART_IT_TXE);
    }
}

void Usart6_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
	  count=0;
    for ( i = 0; i < data_num; i++ )
    {
        TxBuffer[count++] = * ( DataToSend + i );
    }

    if ( ! ( USART6->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( USART6, USART_IT_TXE, ENABLE ); //打开发送中断
    }
}

void usart6_sendstring(u8* str){
	int i;
	for(i=0;i<count;i++){
		USART_SendData(USART6,*(str+i));
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET );//等待发送完成
	}
}
