#include "Tim.h"

void Tim_Nvic_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM_Init()
{
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    //Tim_Nvic_Init();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);      //    TIM3 clock enable 

 
    TIM_TimeBaseStructure.TIM_Period = 199;
    TIM_TimeBaseStructure.TIM_Prescaler = 8399;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    

		 
		TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);
    
    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE); 
		TIM_ClearFlag(TIM3,TIM_FLAG_Update); 

}



Gpio_Info Gpio_info[KEY_NUMBER]={
    {LED1_RCC,LED1_GPIO,LED1_GPIO_PIN},
    {LED2_RCC,LED2_GPIO,LED2_GPIO_PIN},
    {LED3_RCC,LED3_GPIO,LED3_GPIO_PIN},
    {LED4_RCC,LED4_GPIO,LED4_GPIO_PIN}
};
/********************************************************************************************
*函数名称：void LEDGpio_Init(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：LED初始化
*******************************************************************************************/
void LEDGpio_Init(void)
{
    uint8_t i;
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    for(i=0;i<KEY_NUMBER;i++)
    {
        RCC_AHB1PeriphClockCmd( Gpio_info[i].rcc , ENABLE);
        GPIO_InitStructure.GPIO_Pin = Gpio_info[i].pin;
        GPIO_Init(Gpio_info[i].gpio, &GPIO_InitStructure);
    }
}

void LED1_On(void)
{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

void LED1_Off(void)
{
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);	
}
