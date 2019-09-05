#ifndef APP_CFG_H
#define APP_CFG_H

#define LED_EN 0u
#define TIM_LED_EN 0u
#define PWM_LED_EN 0u
#define GPIO_EN 1u
#define NVIC_EN 1u
#define USART_EN 1u
#define PWM_EN 0u
#define I2C_EN 1u
#define MPU9150_EN 1u
#define DELAY_EN 0u

#if LED_EN
	#include "led.h"
#endif

#if TIM_LED_EN
	#include "tim_led.h"
#endif

#if PWM_LED_EN
	#include "pwm_led.h"
#endif

#if GPIO_EN
	#include "gpio.h"
#endif

#if NVIC_EN
	#include "nvic.h"
#endif

#if USART_EN
	#include "usart.h"
#endif

#if PWM_EN
	#include "pwm.h"
#endif

#if I2C_EN
	#include "IIC_Init.h"
#endif

#if MPU9150_EN
	#include "GY86.h"
#endif

#if DELAY_EN
	#include "delay.h"
#endif

#endif
