#include "pwm.h"

void PWM_OUT_INIT(void){
	uint16_t arr = 999;
	uint16_t psc = 1547;//54.27hz 的 PWM
//	//uint32_t lowPulse=54;
//	
	RCC->APB1ENR|=1<<1; //TIM3 时钟使能
	TIM3->CR1&=~(3<<5);//边沿对齐模式
	TIM3->CR1&=~(1<<4);//计数器递增计数
	TIM3->SMCR &= ~(7<<0);//时钟来源设置为内部时钟             /////////////////////
	TIM3->ARR=arr; //设定计数器自动重装值
	TIM3->PSC=psc; //预分频器不分频
	
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	
	//TIM3的通道一、二初始化
	RCC->AHB1ENR|=1<<0; //使能 PORTA 时钟
	GPIO_Set(GPIOA,GPIO_Pin_6,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Fast_Speed,GPIO_PuPd_NOPULL);//复用功能,无上拉下拉输出，低速
	GPIO_Set(GPIOA,GPIO_Pin_7,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Fast_Speed,GPIO_PuPd_NOPULL);//复用功能,无上拉下拉输出，低速
	GPIO_AF_Set(GPIOA,6,2); //PA6,AF2
	GPIO_AF_Set(GPIOA,7,2); //PA7,AF2
	TIM3->CCMR1&=~(3<<0);//CH1 设置为输出模式            ////////////////////////
	TIM3->CCMR1|=6<<4; //CH1 PWM1 模式
	TIM3->CCMR1|=1<<3; //CH1 预装载使能
	TIM3->CCER|=1<<0; //OC1 输出使能
	TIM3->CCER&=~(1<<1); //OC1 高电平有效
	//TIM3->CCER|=1<<1; //OC1 低电平有效
	TIM3->CCMR1&=~(3<<8);//CH2 设置为输出模式            ////////////////////////
	TIM3->CCMR1|=6<<12; //CH2 PWM1 模式
	TIM3->CCMR1|=1<<11; //CH2 预装载使能
	TIM3->CCER|=1<<4; //OC2 输出使能
	TIM3->CCER&=~(1<<5); //OC2 高电平有效
	//TIM3->CCER|=1<<5; //OC2 低电平有效
	
	//TIM3的通道三、四初始化
	RCC->AHB1ENR|=1<<1; //使能 PORTB 时钟
	GPIO_Set(GPIOB,GPIO_Pin_0,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Fast_Speed,GPIO_PuPd_NOPULL);//复用功能,无上拉下拉输出，低速
	GPIO_Set(GPIOB,GPIO_Pin_1,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Fast_Speed,GPIO_PuPd_NOPULL);//复用功能,无上拉下拉输出，低速
	GPIO_AF_Set(GPIOB,0,2); //PB0,AF2
	GPIO_AF_Set(GPIOB,1,2); //PB1,AF2
	TIM3->CCMR2&=~(3<<0);//CH3 设置为输出模式            ////////////////////////
	TIM3->CCMR2|=6<<4; //CH3 PWM1 模式
	TIM3->CCMR2|=1<<3; //CH3 预装载使能
	TIM3->CCER|=1<<8; //OC3 输出使能
	TIM3->CCER&=~(1<<9); //OC3 高电平有效
	//TIM3->CCER|=1<<9; //OC3 低电平有效
	TIM3->CCMR2&=~(3<<8);//CH2 设置为输出模式            ////////////////////////
	TIM3->CCMR2|=6<<12; //CH2 PWM1 模式
	TIM3->CCMR2|=1<<11; //CH2 预装载使能
	TIM3->CCER|=1<<12; //OC4 输出使能
	TIM3->CCER&=~(1<<13); //OC4 高电平有效	
	//TIM3->CCER|=1<<13; //OC4 低电平有效
	
	
	
	TIM3->CR1|=1<<7; //ARPE 使能
	TIM3->CR1|=1<<0; //使能定时器 3

//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef TIM_OCInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//TIM3 时钟使能
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能 PORTA 时钟
//	
//	GPIO_Set(GPIOA,GPIO_Pin_6,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Fast_Speed,GPIO_PuPd_NOPULL);//复用功能,无上拉下拉输出，低速
//	GPIO_AF_Set(GPIOA,6,2); //PA6,AF2
//	
//	TIM_TimeBaseStructure.TIM_Prescaler=psc; //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr; //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器 3
//	
//	//初始化 TIM3 Channel1 PWM 模式
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM 调制模式 1
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性低
//	TIM_OC1Init(TIM3, &TIM_OCInitStructure); //初始化外设 TIM3 OC1
//	
//	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable); //使能预装载寄存器
//	
//	TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE 使能
//	TIM_Cmd(TIM3, ENABLE); //使能 TIM3
}

void TIM5_CH1_Cap_Init(void){
	uint32_t arr = 0xffffffff;
	uint16_t psc = 83;//以1Mhz的频率计数
	
	RCC->APB1ENR|=1<<3; //TIM5 时钟使能
	RCC->AHB1ENR|=1<<0; //使能 PORTA 时钟
	//GPIO_Set(GPIOA,PIN0,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PD);//复用功能,下拉
	GPIO_Set(GPIOA,GPIO_Pin_0,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Fast_Speed,GPIO_PuPd_DOWN);
	GPIO_AF_Set(GPIOA,0,2); //PA0,AF2
	TIM5->ARR=arr; //设定计数器自动重装值
	TIM5->PSC=psc; //预分频器
	TIM5->CCMR1|=1<<0; //CC1S=01 选择输入端 IC1 映射到 TI1 上
	TIM5->CCMR1|=0<<4; //IC1F=0000 配置输入滤波器 不滤波
	TIM5->CCMR1|=0<<10; //IC1PS=00 配置输入分频,不分频
	TIM5->CCER|=0<<1; //CC1P=0 上升沿捕获
	TIM5->CCER|=1<<0; //CC1E=1 允许捕获计数器的值到捕获寄存器中
	TIM5->EGR=1<<0; //软件控制产生更新事件,使写入 PSC 的值立即生效,
	//否则将会要等到定时器溢出才会生效!
	TIM5->DIER|=1<<1; //允许捕获 1 中断
	TIM5->DIER|=1<<0; //允许更新中断
	TIM5->CR1|=0x01; //使能定时器 2
	MY_NVIC_Init(2,0,TIM5_IRQn,2);//抢占 2，子优先级 0，组 2
}

