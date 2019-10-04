#ifndef GPIO_H
#define GPIO_H
#include "includes.h"
//正点原子——《STM32F4开发指南-寄存器版本_V1.1》

//Functions definition
void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD);
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx);

#endif
