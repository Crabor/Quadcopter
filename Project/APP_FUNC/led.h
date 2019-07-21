#ifndef __LED_H__
#define __LED_H__
#include "stm32f4xx_gpio.h"

void LED_INIT(uint32_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed,GPIOOType_TypeDef GPIO_OType,GPIOPuPd_TypeDef GPIO_PuPd);

#endif
