#ifndef PWM_H
#define PWM_H
#include "includes.h"

//输入捕获定时器（TIM5）频率为 1MHz ,即 1/1M 秒计数器加1，
//则一次上升沿捕获到下降沿捕获的时间为 x/1M 秒，x为两次捕获计数器值相减
//又遥控器接收器的PWM频率为 54.27Hz ，即一个PWM持续时间为 1/54.27 秒
//假设此时输入的占空比为D，则 D / 54.27 = x / 1M => x = 1000000D / 54.27
//又因为PWM输出定时器（TIM3）arr为1000，为了输出的占空比和输入的一样，则所需的CCRx值为 1000D
//于是 x * PWM_IN_TO_OUT = 1000D => PWM_IN_TO_OUT = 1000D * 54.27 / 1000000D = 0.05427
#define PWM_IN_TO_OUT 0.05427f

extern int16_t motor1, motor2, motor3, motor4; //四个电机速度
extern u16 PWM_IN_CH[4];

void PWM_IN_Init(void);
void PWM_OUT_Init(void);
void TIM5_PWM_IN_IRQ(void);
void PWM_OUT(void);

#endif
