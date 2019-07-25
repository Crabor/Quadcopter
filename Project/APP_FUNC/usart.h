#ifndef USART_H
#define USART_H
//#include "stm32f4xx_usart.h"
//#include "gpio.h"
//#include "nvic.h"
#include "stdio.h"
#include "includes.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口2接收
#define EN_USART6_RX 			1		//使能（1）/禁止（0）串口6接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	

void uart2_init(u32 pclk2,u32 bound); 
void uart6_init(u32 pclk2,u32 bound); 

#endif
