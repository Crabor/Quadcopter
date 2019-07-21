#include "gpio.h"

//GPIO 通用设置
//GPIOx:GPIOA~GPIOI.
//BITx:0X0000~0XFFFF,位设置,每个位代表一个 IO,
//第 0 位代表 Px0,第 1 位代表 Px1,依次类推.比如 0X0101,代表同时设置 Px0 和 Px8.
//MODE:0~3;模式选择,0,输入(系统复位默认状态);1,普通输出;2,复用功能;3,模拟输入.
//OTYPE:0/1;输出类型选择,0,推挽输出;1,开漏输出.
//OSPEED:0~3;输出速度设置,0,2Mhz;1,25Mhz;2,50Mhz;3,100Mh.
//PUPD:0~3:上下拉设置,0,不带上下拉;1,上拉;2,下拉;3,保留.
//注意:在输入模式(普通输入/模拟输入)下,OTYPE 和 OSPEED 参数无效!!
void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD)
{
	u32 pinpos=0,pos=0,curpin=0;
	for(pinpos=0;pinpos<16;pinpos++)
	{
		pos=1<<pinpos; //一个个位检查
		curpin=BITx&pos;//检查引脚是否要设置
		if(curpin==pos) //需要设置
		{
			GPIOx->MODER&=~(3<<(pinpos*2)); //先清除原来的设置
			GPIOx->MODER|=MODE<<(pinpos*2); //设置新的模式
		if((MODE==0X01)||(MODE==0X02)) //如果是输出模式/复用功能模式
		{
			GPIOx->OSPEEDR&=~(3<<(pinpos*2)); //清除原来的设置
			GPIOx->OSPEEDR|=(OSPEED<<(pinpos*2));//设置新的速度值
			GPIOx->OTYPER&=~(1<<pinpos) ; //清除原来的设置
			GPIOx->OTYPER|=OTYPE<<pinpos; //设置新的输出模式
		}
		GPIOx->PUPDR&=~(3<<(pinpos*2)); //先清除原来的设置
		GPIOx->PUPDR|=PUPD<<(pinpos*2); //设置新的上下拉
		}
	}
}

//GPIO 复用设置
//GPIOx:GPIOA~GPIOI.
//BITx:0~15,代表 IO 引脚编号.
//AFx:0~15,代表 AF0~AF15.
//AF0~15 设置情况(这里仅是列出常用的,详细的请见 407 数据手册,56 页 Table 7):
//AF0:MCO/SWD/ SWCLK/RTC AF1:TIM1/2; AF2:TIM3~5; AF3:TIM8~11
//AF4:I2C1~I2C3; AF5:SPI1/SPI2; AF6:SPI3; AF7:USART1~3;
//AF8:USART4~6; AF9;CAN1/2/TIM12~14 AF10:USB_OTG/USB_HS AF11:ETH
//AF12:FSMC/SDIO/OTG/HS AF13:DCIM AF14: AF15:EVENTOUT
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx)
{
	GPIOx->AFR[BITx>>3]&=~(0X0F<<((BITx&0X07)*4));
	GPIOx->AFR[BITx>>3]|=(u32)AFx<<((BITx&0X07)*4);
}
