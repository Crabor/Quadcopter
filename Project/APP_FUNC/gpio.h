#ifndef GPIO_H
#define GPIO_H
//#include "stm32f4xx_gpio.h"
#include "includes.h"

void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD);
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx);

#endif
