#ifndef NVIC_H
#define NVIC_H
#include "includes.h"
//正点原子——《STM32F4开发指南-寄存器版本_V1.1》

//函数定义
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channe, u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM);
#endif
