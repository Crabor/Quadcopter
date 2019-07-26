#include "usart.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////

							 
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
	#if EN_USART6_RX		  	//如果使能了接收
		//使能接收中断 
		USART6->CR1|=1<<2;  	//串口接收使能
		USART6->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(3,3,USART6_IRQn,2);//组2，最低优先级 
	#endif
		USART6->CR1|=1<<13;  	//串口使能
}


