#include "pwm.h"

void PWM_OUT_Init(void)
{
    //TODO 是否要保持定时器PWM输出频率和遥控器接收器PWM输出频率一致？
    uint16_t arr = 1000 - 1;
    uint16_t psc = 1548 - 1; //54.27hz 的 PWM
    //	uint16_t arr = 2500-1;
    //	uint16_t psc = 84-1;//400hz 的 PWM

    RCC->APB1ENR |= 1 << 1; //TIM3 时钟使能
    TIM3->CR1 &= ~(3 << 5); //边沿对齐模式
    TIM3->CR1 &= ~(1 << 4); //计数器递增计数
    TIM3->SMCR &= ~(7 << 0); //时钟来源设置为内部时钟             /////////////////////
    TIM3->ARR = arr; //设定计数器自动重装值
    TIM3->PSC = psc; //预分频器不分频

    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    //TIM3的通道一、二初始化
    RCC->AHB1ENR |= 1 << 0; //使能 PORTA 时钟
    GPIO_Set(GPIOA, GPIO_Pin_6, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL); //复用功能,无上拉下拉输出，低速
    GPIO_Set(GPIOA, GPIO_Pin_7, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL); //复用功能,无上拉下拉输出，低速
    GPIO_AF_Set(GPIOA, 6, 2); //PA6,AF2
    GPIO_AF_Set(GPIOA, 7, 2); //PA7,AF2
    TIM3->CCMR1 &= ~(3 << 0); //CH1 设置为输出模式            ////////////////////////
    TIM3->CCMR1 |= 6 << 4; //CH1 PWM1 模式
    TIM3->CCMR1 |= 1 << 3; //CH1 预装载使能
    TIM3->CCER |= 1 << 0; //OC1 输出使能
    TIM3->CCER &= ~(1 << 1); //OC1 高电平有效
    //TIM3->CCER|=1<<1; //OC1 低电平有效
    TIM3->CCMR1 &= ~(3 << 8); //CH2 设置为输出模式            ////////////////////////
    TIM3->CCMR1 |= 6 << 12; //CH2 PWM1 模式
    TIM3->CCMR1 |= 1 << 11; //CH2 预装载使能
    TIM3->CCER |= 1 << 4; //OC2 输出使能
    TIM3->CCER &= ~(1 << 5); //OC2 高电平有效
    //TIM3->CCER|=1<<5; //OC2 低电平有效

    //TIM3的通道三、四初始化
    RCC->AHB1ENR |= 1 << 1; //使能 PORTB 时钟
    GPIO_Set(GPIOB, GPIO_Pin_0, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL); //复用功能,无上拉下拉输出，低速
    GPIO_Set(GPIOB, GPIO_Pin_1, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL); //复用功能,无上拉下拉输出，低速
    GPIO_AF_Set(GPIOB, 0, 2); //PB0,AF2
    GPIO_AF_Set(GPIOB, 1, 2); //PB1,AF2
    TIM3->CCMR2 &= ~(3 << 0); //CH3 设置为输出模式            ////////////////////////
    TIM3->CCMR2 |= 6 << 4; //CH3 PWM1 模式
    TIM3->CCMR2 |= 1 << 3; //CH3 预装载使能
    TIM3->CCER |= 1 << 8; //OC3 输出使能
    TIM3->CCER &= ~(1 << 9); //OC3 高电平有效
    //TIM3->CCER|=1<<9; //OC3 低电平有效
    TIM3->CCMR2 &= ~(3 << 8); //CH2 设置为输出模式            ////////////////////////
    TIM3->CCMR2 |= 6 << 12; //CH2 PWM1 模式
    TIM3->CCMR2 |= 1 << 11; //CH2 预装载使能
    TIM3->CCER |= 1 << 12; //OC4 输出使能
    TIM3->CCER &= ~(1 << 13); //OC4 高电平有效
    //TIM3->CCER|=1<<13; //OC4 低电平有效

    TIM3->CR1 |= 1 << 7; //ARPE 使能
    TIM3->CR1 |= 1 << 0; //使能定时器 3
}

void PWM_IN_Init(void)
{
//     uint32_t arr = 0xffff;
//     uint16_t psc = 84 - 1; //以1Mhz的频率计数，每0xffffus发生一次计数器更新中断

//     RCC->APB1ENR |= 1 << 3; //TIM5 时钟使能
//     RCC->AHB1ENR |= 1 << 0; //使能 PORTA 时钟
//     //GPIO_Set(GPIOA,PIN0,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PD);//复用功能,下拉

//     //TIM5->CH1   PA0
//     GPIO_Set(GPIOA, GPIO_Pin_0, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_UP);
//     GPIO_AF_Set(GPIOA, 0, 2); //PA0,AF2
//     //TIM5->CH2   PA1
//     GPIO_Set(GPIOA, GPIO_Pin_1, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_UP);
//     GPIO_AF_Set(GPIOA, 1, 2); //PA1,AF2
//     //TIM5->CH3   PA2
//     GPIO_Set(GPIOA, GPIO_Pin_2, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_UP);
//     GPIO_AF_Set(GPIOA, 2, 2); //PA2,AF2
//     //TIM5->CH4   PA3
//     GPIO_Set(GPIOA, GPIO_Pin_3, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_UP);
//     GPIO_AF_Set(GPIOA, 3, 2); //PA3,AF2

//     TIM5->ARR = arr; //设定计数器自动重装值
//     TIM5->PSC = psc; //预分频器

//     //TIM5->CH1   PA0
//     TIM5->CCMR1 |= 1 << 0; //CC1S=01 选择输入端 IC1 映射到 TI1 上
//     TIM5->CCMR1 |= 0 << 4; //IC1F=0000 配置输入滤波器 不滤波
//     TIM5->CCMR1 |= 0 << 2; //IC1PSC=00 配置输入分频,不分频
//     TIM5->CCER |= 0 << 1; //CC1P=0 上升沿捕获
//     TIM5->CCER |= 1 << 0; //CC1E=1 允许捕获计数器的值到捕获寄存器中
//     //TIM5->CH2   PA1
//     TIM5->CCMR1 |= 1 << 8; //CC2S=01 选择输入端 IC2 映射到 TI2 上
//     TIM5->CCMR1 |= 0 << 12; //IC2F=0000 配置输入滤波器 不滤波
//     TIM5->CCMR1 |= 0 << 10; //IC2PSC=00 配置输入分频,不分频
//     TIM5->CCER |= 0 << 5; //CC2P=0 上升沿捕获
//     TIM5->CCER |= 1 << 4; //CC2E=1 允许捕获计数器的值到捕获寄存器中
//     //TIM5->CH3   PA2
//     TIM5->CCMR2 |= 1 << 0; //CC3S=01 选择输入端 IC3 映射到 TI3 上
//     TIM5->CCMR2 |= 0 << 4; //IC3F=0000 配置输入滤波器 不滤波
//     TIM5->CCMR2 |= 0 << 2; //IC3PSC=00 配置输入分频,不分频
//     TIM5->CCER |= 0 << 9; //CC3P=0 上升沿捕获
//     TIM5->CCER |= 1 << 8; //CC3E=1 允许捕获计数器的值到捕获寄存器中
//     //TIM5->CH4   PA3
//     TIM5->CCMR2 |= 1 << 8; //CC4S=01 选择输入端 IC4 映射到 TI4 上
//     TIM5->CCMR2 |= 0 << 12; //IC4F=0000 配置输入滤波器 不滤波
//     TIM5->CCMR2 |= 0 << 10; //IC4PSC=00 配置输入分频,不分频
//     TIM5->CCER |= 0 << 13; //CC4P=0 上升沿捕获
//     TIM5->CCER |= 1 << 12; //CC4E=1 允许捕获计数器的值到捕获寄存器中

//     TIM5->EGR = 1 << 0; //软件控制产生更新事件,使写入 PSC 的值立即生效,
//     //否则将会要等到定时器溢出才会生效!
		
//     TIM5->DIER |= 0x0F << 1; //允许捕获 1、2、3、4 中断
// //    TIM5->DIER |= 1 << 3;
		
//     // TIM5->SR &= ~(1 << 0); //清除更新中断标志位
//     // TIM5->DIER |= 1 << 0; //允许更新中断
//     TIM5->CR1 |= 0x01; //使能定时器 5
//     MY_NVIC_Init(3, 3, TIM5_IRQn, 2); //组2,抢占优先级3，响应优先级0


       GPIO_InitTypeDef GPIO_InitStructure;
       NVIC_InitTypeDef NVIC_InitStructure;
       TIM_ICInitTypeDef TIM_ICInitStructure;
       TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

       // Enable GPIOA and TIM5 clock
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

       // GPIO configuration. Push-pull alternate function
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
       GPIO_Init(GPIOA, &GPIO_InitStructure);

       // GPIO alternate function configuration
       GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
       GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
       GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
       GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

       // Close TIM5
       TIM_DeInit(TIM5);
       // TIM5 configuration. Prescaler is 84, period is 0xFFFF, and counter mode is up
       TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
       TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
       TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
       TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
       TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

       // TIM5 input configuration. Capture on rising edge and filter value is 0x0B
       TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
       TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
       TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
       TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
       TIM_ICInitStructure.TIM_ICFilter = 0x0B;
       TIM_ICInit(TIM5, &TIM_ICInitStructure);

       TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
       TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
       TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
       TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
       TIM_ICInitStructure.TIM_ICFilter = 0x0B;
       TIM_ICInit(TIM5, &TIM_ICInitStructure);

       TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
       TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
       TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
       TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
       TIM_ICInitStructure.TIM_ICFilter = 0x0B;
       TIM_ICInit(TIM5, &TIM_ICInitStructure);

       TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
       TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
       TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
       TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
       TIM_ICInitStructure.TIM_ICFilter = 0x0B;
       TIM_ICInit(TIM5, &TIM_ICInitStructure);

       // NVIC initialization
       NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);

       // TIM5 interrupt configuration
       TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
       TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
       TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);
       TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);

       // Enable TIM5
       TIM_Cmd(TIM5, ENABLE);
}

// Capture status of channels
unsigned char TIM5CH1_CAPTURE_STA = 1;
unsigned char TIM5CH2_CAPTURE_STA = 1;
unsigned char TIM5CH3_CAPTURE_STA = 1;
unsigned char TIM5CH4_CAPTURE_STA = 1;

// Rising edge/falling edge data
uint16_t TIM5CH1_Rise, TIM5CH1_Fall,
    TIM5CH2_Rise, TIM5CH2_Fall,
    TIM5CH3_Rise, TIM5CH3_Fall,
    TIM5CH4_Rise, TIM5CH4_Fall;

// Overflow processing variable
uint16_t TIM5_T;

// Four-channel remote control initial value
extern uint16_t PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4;

void TIM5_PWM_IN_IRQ(void)
{
	SendStr("ok1");
    //	  static u32 cnt=0;
    //		cnt++;
    //		SendWord(&cnt);
    // CH1 - AIL - Roll
    // Capture the interrupt
    if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) {
        // Clear interrupt flag
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
        // Capture the rising edge
        if (TIM5CH1_CAPTURE_STA == 1) {
            // Get the data of rising edge
            TIM5CH1_Rise = TIM_GetCapture1(TIM5);
            // Change capture status to falling edge
            TIM5CH1_CAPTURE_STA = 0;
            // Set to capture on falling edge
            TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);
        }
        // Capture the falling edge
        else {
            // Get the data of falling edge
            TIM5CH1_Fall = TIM_GetCapture1(TIM5);
            // Change capture status to rising edge
            TIM5CH1_CAPTURE_STA = 1;

            // Overflow processing
            if (TIM5CH1_Fall < TIM5CH1_Rise) {
                TIM5_T = 65535;
            } else {
                TIM5_T = 0;
            }

            // Falling edge time minus rising edge time to get high-level time
            PWMInCh1 = TIM5CH1_Fall - TIM5CH1_Rise + TIM5_T;
            // Set to capture on rising edge
            TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);
        }
    }

    // CH2 - ELE - Pitch
    if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);

        if (TIM5CH2_CAPTURE_STA == 1) {
            TIM5CH2_Rise = TIM_GetCapture2(TIM5);
            TIM5CH2_CAPTURE_STA = 0;
            TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Falling);
        } else {
            TIM5CH2_Fall = TIM_GetCapture2(TIM5);
            TIM5CH2_CAPTURE_STA = 1;
            if (TIM5CH2_Fall < TIM5CH2_Rise) {
                TIM5_T = 65535;
            } else {
                TIM5_T = 0;
            }
            PWMInCh2 = TIM5CH2_Fall - TIM5CH2_Rise + TIM5_T;
            TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Rising);
        }
    }

    // CH3 - THR - Acc
    if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET) {
        //        SendStr("ok1");
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
        if (TIM5CH3_CAPTURE_STA == 1) {
            TIM5CH3_Rise = TIM_GetCapture3(TIM5);
            TIM5CH3_CAPTURE_STA = 0;
            TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Falling);
        } else {
            TIM5CH3_Fall = TIM_GetCapture3(TIM5);
            TIM5CH3_CAPTURE_STA = 1;
            if (TIM5CH3_Fall < TIM5CH3_Rise) {
                TIM5_T = 65535;
            } else {
                TIM5_T = 0;
            }
            PWMInCh3 = TIM5CH3_Fall - TIM5CH3_Rise + TIM5_T;
            TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Rising);
        }
    }

    // CH4 - RUD -Yaw
    if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);

        if (TIM5CH4_CAPTURE_STA == 1) {
            TIM5CH4_Rise = TIM_GetCapture4(TIM5);
            TIM5CH4_CAPTURE_STA = 0;
            TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Falling);
        } else {
            TIM5CH4_Fall = TIM_GetCapture4(TIM5);
            TIM5CH4_CAPTURE_STA = 1;
            if (TIM5CH4_Fall < TIM5CH4_Rise) {
                TIM5_T = 65535;
            } else {
                TIM5_T = 0;
            }
            PWMInCh4 = TIM5CH4_Fall - TIM5CH4_Rise + TIM5_T;
            TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Rising);
        }
    }
    //		TIM3->CCR1=PWMInCh1*0.054;//PWM输出
    //		TIM3->CCR2=PWMInCh2*0.054;//PWM输出
    //		TIM3->CCR3=PWMInCh3*0.054;//PWM输出
    //		TIM3->CCR4=PWMInCh4*0.054;//PWM输出

    //			TIM3->CCR1=PWMInCh1*0.054;//PWM输出
    //			TIM3->CCR2=PWMInCh3*0.054;//PWM输出
    //			TIM3->CCR3=PWMInCh3*0.054;//PWM输出
    //			TIM3->CCR4=PWMInCh3*0.054;//PWM输出

//    SendPWMIN(0xF1, &TIM5CH1_CAPTURE_STA, &TIM5_T, &TIM5CH1_Rise, &TIM5CH1_Fall, &PWMInCh1);
//    SendPWMIN(0xF2, &TIM5CH2_CAPTURE_STA, &TIM5_T, &TIM5CH2_Rise, &TIM5CH2_Fall, &PWMInCh2);
    SendPWMIN(0xF3, &TIM5CH3_CAPTURE_STA, &TIM5_T, &TIM5CH3_Rise, &TIM5CH3_Fall, &PWMInCh3);
//    SendPWMIN(0xF4, &TIM5CH4_CAPTURE_STA, &TIM5_T, &TIM5CH4_Rise, &TIM5CH4_Fall, &PWMInCh4);
}

//// Capture status of channels
//unsigned char TIM5CH1_CAPTURE_STA = 1;
//unsigned char TIM5CH2_CAPTURE_STA = 1;
//unsigned char TIM5CH3_CAPTURE_STA = 1;
//unsigned char TIM5CH4_CAPTURE_STA = 1;

//// Rising edge/falling edge data
//uint16_t TIM5CH1_Rise, TIM5CH1_Fall,
//         TIM5CH2_Rise, TIM5CH2_Fall,
//         TIM5CH3_Rise, TIM5CH3_Fall,
//         TIM5CH4_Rise, TIM5CH4_Fall;

//// Overflow processing variable
//uint16_t TIM5_T;

//// Four-channel remote control initial value
//uint16_t PWMInCh1 = 0, PWMInCh2 = 0, PWMInCh3 = 0, PWMInCh4 = 0;

////TIM5_CAPTURE_STA[i]:0,还没捕获到高电平;1,已经捕获到高电平了.
////TIM5_CAPTURE_OVF[i]:0,未溢出;1,溢出.
////TIM5_CAPTURE_VAL[i][0]:上升沿对应计数器值
////TIM5_CAPTURE_VAL[i][1]:下降沿对应计数器值
////PWM_IN_CH[i]:通道i+1的输入PWM位宽
//u8 TIM5_CAPTURE_STA[4];
//u8 TIM5_CAPTURE_OVF[4];
//u32 TIM5_CAPTURE_VAL[4][2];
//u32 PWM_IN_CH[4];
//void TIM5_PWM_IN_IRQ(void){
//  u8 i;
////	SendString("Enter IT\r\n");
//  //定时器计数器溢出更新中断
//  if(TIM5->SR&0x01){
////		TIM5->SR&=~(1<<0);//清除中断标志
//		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);//清除中断标志
//    for(i=0;i<4;i++){
//      if(TIM5_CAPTURE_STA[i])//通道i+1正在进行输入捕获
//        TIM5_CAPTURE_OVF[i]++; //溢出次数加一
//    }
//  }
//
//  //通道i+1中断
//  for(i=0;i<4;i++){
//    if(TIM5->SR&(1<<(i+1))){
////			TIM5->SR&=~((0x0001<<(i+1));//清除中断标志
//			TIM_ClearITPendingBit(TIM5, 0x0002<<i);//清除中断标志
//      if(TIM5_CAPTURE_STA[i]){//已捕获到高电平，说明此时捕获到下降沿
//        TIM5_CAPTURE_STA[i]=0;//更改捕获状态
//        TIM5_CAPTURE_VAL[i][1]=*(&(TIM5->CCR1)+i);//读取下降沿对应计数器值
//				PWM_IN_CH[i]=TIM5_CAPTURE_VAL[i][1]-TIM5_CAPTURE_VAL[i][0]+TIM5_CAPTURE_OVF[i]*0xffff;//计算脉冲宽度
//				TIM5_CAPTURE_OVF[i]=0;//溢出次数清零
////        TIM5->CCER&=~(0x0001<<(1+4*i)); //CCxP=00 通道上升沿捕获
//				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);//通道上升沿捕获
//      }else{//捕获到上升沿
//        TIM5_CAPTURE_STA[i]=1;//更改捕获状态
//        TIM5_CAPTURE_VAL[i][0]=*(&(TIM5->CCR1)+i);//读取上升沿对应计数器值
////        TIM5->CCER|=(0x01<<(1+4*i)); //CCxP=01 通道下降沿捕获
//				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);//通道下降沿捕获
//      }
//    }
//  }
//	SendPWMIN(&TIM5_CAPTURE_STA[0],&TIM5_CAPTURE_OVF[0],&TIM5_CAPTURE_VAL[0][0],&TIM5_CAPTURE_VAL[0][1],&PWM_IN_CH[0]);
////	SendHalfWord(&(TIM5->SR));
//}
