#include "includes.h"
//硬中断，系统初始化前延时用以。
//与OSTimeDly()不同，不会引起系统调度，且支持微妙数

// Functions definition
void DELAY_Init(u8 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u16 nms);
