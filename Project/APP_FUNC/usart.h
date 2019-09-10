#ifndef USART_H
#define USART_H
//#include "stm32f4xx_usart.h"
//#include "gpio.h"
//#include "nvic.h"
#include "stdio.h"
#include "includes.h"

extern u8 Rx_Buf[];

void uart2_init(u32 pclk2,u32 bound); 
void uart6_init(u32 pclk2,u32 bound);

void Usart6_IRQ ( void );
void Usart6_Send ( unsigned char *DataToSend , u8 data_num );
void usart6_sendstring(u8* str);
#endif
