#ifndef __DELAY_H_
#define __DELAY_H_

#include "includes.h"
//使用SysTick的普通计数模式对延迟进行管理

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif
