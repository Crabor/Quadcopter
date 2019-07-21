#include "nvic.h"

void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
	u32 temp,temp1;
	temp1=(~NVIC_Group)&0x07;//取后三位
	temp1<<=8;
	temp=SCB->AIRCR; //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
	temp|=0X05FA0000; //注册密钥。该域用来防止对该寄存器的意外写入操作。改变该寄存器的位之前必须将 0x05FA 写入该域。
	temp|=temp1;
	SCB->AIRCR=temp; //设置分组
}

//设置 NVIC
//NVIC_PreemptionPriority： 抢占优先级
//NVIC_SubPriority ： 响应优先级
//NVIC_Channel ： 中断编号
//NVIC_Group ： 中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分：
//组 0： 0 位抢占优先级， 4 位响应优先级
//组 1： 1 位抢占优先级， 3 位响应优先级
//组 2： 2 位抢占优先级， 2 位响应优先级
//组 3： 3 位抢占优先级， 1 位响应优先级
//组 4： 4 位抢占优先级， 0 位响应优先级
//NVIC_SubPriority 和 NVIC_PreemptionPriority 的原则是， 数值越小， 越优先
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)
{
	u32 temp;
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf; //取低四位
	NVIC->ISER[NVIC_Channel/32]|=1<<NVIC_Channel%32;//使能中断位(要清除的话,设置 ICER 对应位为 1 即可)
	NVIC->IP[NVIC_Channel]|=temp<<4; //设置响应优先级和抢断优先级
}

//外部中断配置函数
//只针对 GPIOA~I;不包括 PVD,RTC,USB_OTG,USB_HS,以太网唤醒等
//参数:
//GPIOx:0~8,代表 GPIOA~I
//BITx:需要使能的位;
//TRIM:触发模式,1,下升沿;2,上降沿;3，任意电平触发
//该函数一次只能配置 1 个 IO 口,多个 IO 口,需多次调用
//该函数会自动开启对应中断,以及屏蔽线
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM)
{
	u8 EXTOFFSET=(BITx%4)*4;
	RCC->APB2ENR|=1<<14; //使能 SYSCFG 时钟
	SYSCFG->EXTICR[BITx/4]&=~(0x000F<<EXTOFFSET); //清除原来设置！！！
	SYSCFG->EXTICR[BITx/4]|=GPIOx<<EXTOFFSET;
	//EXTI.BITx 映射到 GPIOx.BITx
	//自动设置
	EXTI->IMR|=1<<BITx; //开启 line BITx 上的中断(如果要禁止中断，则反操作即可)
	if(TRIM&0x01)EXTI->FTSR|=1<<BITx; //line BITx 上事件下降沿触发
	if(TRIM&0x02)EXTI->RTSR|=1<<BITx; //line BITx 上事件上升降沿触发
}
