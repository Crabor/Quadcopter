#include "led.h"

void LED_INIT(uint32_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed,GPIOOType_TypeDef GPIO_OType,GPIOPuPd_TypeDef GPIO_PuPd){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//enable GPIOA clock
	RCC->AHB1ENR |= 1u;
	//reset PA5
	GPIOA->BSRRH |= 0x0020u;
	//set mode:output,push pull,speed low,no pull up or down
  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed;
  GPIO_InitStruct.GPIO_OType = GPIO_OType;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
}
