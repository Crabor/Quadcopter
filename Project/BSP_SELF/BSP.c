#include "includes.h"

void BSP_Init(void){
  SysTick_Init();
	Delay_Config(84);
	uart6_init(84,115200);
//	ANO_DT_SendString("1");
	I2C_Congiguration();
//	ANO_DT_SendString("3");
	MPU6050_Init();
//	while(MPU6050_Init()!=1);//若MPU6050初始化不成功，则程序不向下运行;
//	ANO_DT_SendString("4");
//	Calculate_FilteringCoefficient(0.001f,10.f);//计算IIR滤波器参数
//	PWM_OUT_INIT();
//	PWM_IN_INIT();
}

void SysTick_Init(void){
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.SYSCLK_Frequency/OS_TICKS_PER_SEC);
}

