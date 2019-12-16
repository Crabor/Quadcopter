#ifndef APP_CFG_H
#define APP_CFG_H

//选择编译开关
//串口发送是否采用中断形式
#define USART_IT_EN 0

#define LED_EN 0
#define TIM_LED_EN 0
#define PWM_LED_EN 0
#define GPIO_EN 1
#define NVIC_EN 1
#define USART_EN 1
#define PWM_EN 1
#define I2C_EN 1
#define GY86_EN 1
#define IMU_EN 1
#define DELAY_EN 1
#define PID_EN 1

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

#if GY86_EN
#include "GY86.h"
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
