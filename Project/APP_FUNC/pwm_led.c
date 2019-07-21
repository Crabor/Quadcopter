#include "pwm_led.h"

void PWM_LED_INIT(void){
	uint16_t arr = 499;
	uint16_t psc = 41;//1Mhz 的计数频率,2Khz 的 PWM
	//此部分需手动修改 IO 口设置
	RCC->APB1ENR|=1<<0; //TIM2 时钟使能
	RCC->AHB1ENR|=1<<0; //使能 PORTA 时钟
	GPIO_Set(GPIOA,GPIO_Pin_5,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_100MHz,GPIO_PuPd_UP);//复用功能,上拉输出
	GPIO_AF_Set(GPIOA,5,1); //PA5,AF1
	TIM2->SMCR &= ~(7<<0);//时钟来源设置为内部时钟             /////////////////////
	TIM2->ARR=arr; //设定计数器自动重装值
	TIM2->PSC=psc; //预分频器不分频
	TIM2->CCMR1&=~(3<<0);//CH1 设置为输出模式            ////////////////////////
	TIM2->CCMR1|=6<<4; //CH1 PWM1 模式
	TIM2->CCMR1|=1<<3; //CH1 预装载使能
	TIM2->CCER|=1<<0; //OC1 输出使能
	TIM2->CCER|=1<<1; //OC1 低电平有效
	TIM2->CR1|=1<<7; //ARPE 使能
	TIM2->CR1|=1<<0; //使能定时器 2
}
