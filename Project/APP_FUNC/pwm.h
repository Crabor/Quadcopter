#ifndef PWM_H
#define PWM_H
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "gpio.h"

void PWM_IN_INIT(void);
void PWM_OUT_INIT(void);

#endif
