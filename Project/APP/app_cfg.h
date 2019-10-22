#ifndef APP_CFG_H
#define APP_CFG_H

//选择编译开关
//AK8975还是HMC388L
#define AK8975_EN 0
//串口发送是否采用中断形式
#define USART_IT_EN 0

#define LED_EN 0u
#define TIM_LED_EN 0u
#define PWM_LED_EN 0u
#define GPIO_EN 1u
#define NVIC_EN 1u
#define USART_EN 1u
#define PWM_EN 1u
#define I2C_EN 1u
#define MPU9150_EN 1u
#define IMU_EN 1u
#define DELAY_EN 1u
#define PID_EN 1u

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
	#include "iic.h"
#endif

#if MPU9150_EN
	#include "MPU9150.h"
#endif

#if IMU_EN
	#include "IMU.h"
#endif

#if DELAY_EN
	#include "delay.h"
#endif

#if PID_EN
	#include "pid.h"
#endif

#endif
