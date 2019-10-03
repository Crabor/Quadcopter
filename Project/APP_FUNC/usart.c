#include "usart.h"

//extern u8 testdatatosend[50];	//发送数据缓存
//初始化IO 串口2
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart2_init(u32 pclk2,u32 bound)
{  	 
		float temp;
		u16 mantissa;
		u16 fraction;	   
		temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
		mantissa=temp;				 //得到整数部分
		fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
			mantissa<<=4;
		mantissa+=fraction; 
		RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
		RCC->APB1ENR|=1<<17;  	//使能串口2时钟 
		GPIO_Set(GPIOA,GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP);//PA2,PA3,复用功能,上拉输出
		GPIO_AF_Set(GPIOA,2,7);	//PA2,AF7
		GPIO_AF_Set(GPIOA,3,7);//PA3,AF7  	   
		//波特率设置
		USART2->BRR=mantissa; 	//波特率设置	 
		USART2->CR1&=~(1<<15); 	//设置OVER8=0 
		USART2->CR1|=1<<3;  	//串口发送使能 
	#if EN_USART2_RX		  	//如果使能了接收
		//使能接收中断 
		USART2->CR1|=1<<2;  	//串口接收使能
		USART2->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(3,3,USART2_IRQn,2);//组2，最低优先级 
	#endif
		USART2->CR1|=1<<13;  	//串口使能
}


//初始化IO 串口6
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
void uart6_init(u32 pclk2,u32 bound)
{   //PC6,PC7
    float temp;
    u16 mantissa;
    u16 fraction;
    temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
    mantissa=temp;         //得到整数部分
    fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0
    mantissa<<=4;
    mantissa+=fraction;
    RCC->AHB1ENR|=1<<2; //使能PORTC口时钟
    RCC->APB2ENR|=1<<5; //使能串口6时钟
    GPIO_Set(GPIOC,GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP);//PC6,PC7,复用功能,上拉输出
    GPIO_AF_Set(GPIOC,6,8);  //PC6,AF8
    GPIO_AF_Set(GPIOC,7,8);//PC7,AF8
    //波特率设置
    USART6->BRR=mantissa;   //波特率设置
    USART6->CR1&=~(1<<15);   //设置OVER8=0
    USART6->CR1|=1<<3;    //串口发送使能
    //使能接收中断
//    USART6->CR1|=1<<2;    //串口接收使能
//    USART6->CR1|=1<<5;      //接收缓冲区非空中断使能
    MY_NVIC_Init(2,0,USART6_IRQn,2);//组2,抢占优先级2，响应优先级0
    USART6->CR1|=1<<13;    //串口使能
}


/****************************************串口中断发送方式**********************************************/
u8 FLAG_TC=0;//定义全局变量

void Usart6_IRQ ( void ){
 if (USART_GetITStatus(USART6, USART_IT_TC) != RESET)//发送完成中断,= SET
  {
    USART_ClearITPendingBit(USART6,USART_IT_TC);
    FLAG_TC=1;
  }
}

//DataToSend:发送数组
//data_num:数组长度
void Usart6_Send ( u8 *DataToSend , u8 data_num ){
  u8 i=0;
  FLAG_TC=0;//提前准备一下
  for(i=0;i<data_num;i++)//检测字符串结束符
  {
    USART_SendData(USART6 ,*(DataToSend+i));//发送当前字符
    if(!(USART6->CR1 & USART_CR1_TCIE))//判断USART6->CR1中的TCIE位是否设置，即“发送完成中断”是否使能
    {
        USART_ITConfig(USART6, USART_IT_TC, ENABLE);
    }
    while( FLAG_TC==0);  //0：发送还未完成；1：发送完成
    FLAG_TC=0;
  }
}
/*********************https://blog.csdn.net/qq_35629563/article/details/80879819***********************/

void ANO_DT_SendString(const char *str)
{
	u8 _cnt=0;
	u8 i = 0;
	u8 sum = 0;
	
	testdatatosend[_cnt++]=0xAA;
	testdatatosend[_cnt++]=0x05;
	testdatatosend[_cnt++]=0xAF;
	testdatatosend[_cnt++]=0xA0;
	testdatatosend[_cnt++]=0;
	while(*(str+i) != '\0')
	{
		testdatatosend[_cnt++] = *(str+i);
		i++;
		if(_cnt > 50)
			break;
	}
	
	testdatatosend[4] = _cnt-5;
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;

	Usart6_Send(testdatatosend, _cnt);
}

void SendByte(u8 frame,u8 *p){
	u8 _cnt=0;
	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	int i;
	
	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	testdatatosend[_cnt++]=frame;//0x02，表示本帧为传感器原始数据帧
	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
 
	testdatatosend[_cnt++]=*p;//将要发送的数据放至发送缓冲区
	
	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
}

void SendHalfWord(u8 frame,u16 *p){
	u8 _cnt=0;
	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	int i;
	
	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	testdatatosend[_cnt++]=frame;//0x02，表示本帧为传感器原始数据帧
	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
 
	testdatatosend[_cnt++]=BYTE1(*p);//将要发送的数据放至发送缓冲区
	testdatatosend[_cnt++]=BYTE0(*p);
	
	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
}

void SendWord(u8 frame,u32 *p){
	u8 _cnt=0;
	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	int i;
	
	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	testdatatosend[_cnt++]=frame;//0x02，表示本帧为传感器原始数据帧
	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
 
	testdatatosend[_cnt++]=BYTE3(*p);//将要发送的数据放至发送缓冲区
	testdatatosend[_cnt++]=BYTE2(*p);
	testdatatosend[_cnt++]=BYTE1(*p);
	testdatatosend[_cnt++]=BYTE0(*p);
	
	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
}

void SendSenser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z,int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z,int16_t MAG_X,int16_t MAG_Y,int16_t MAG_Z)	//发送用户数据，这里有6个数据
{
	u8 _cnt=0;
	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	int i;
	
	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	testdatatosend[_cnt++]=0x02;//0x02，表示本帧为传感器原始数据帧
	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

	testdatatosend[_cnt++]=BYTE1(ACCEL_X);//将要发送的数据放至发送缓冲区
	testdatatosend[_cnt++]=BYTE0(ACCEL_X);
	
	testdatatosend[_cnt++]=BYTE1(ACCEL_Y);
	testdatatosend[_cnt++]=BYTE0(ACCEL_Y);
	
	testdatatosend[_cnt++]=BYTE1(ACCEL_Z);
	testdatatosend[_cnt++]=BYTE0(ACCEL_Z);
	
	testdatatosend[_cnt++]=BYTE1(GYRO_X);
	testdatatosend[_cnt++]=BYTE0(GYRO_X);
	
	testdatatosend[_cnt++]=BYTE1(GYRO_Y);
	testdatatosend[_cnt++]=BYTE0(GYRO_Y);
	
	testdatatosend[_cnt++]=BYTE1(GYRO_Z);
	testdatatosend[_cnt++]=BYTE0(GYRO_Z);
	
	testdatatosend[_cnt++]=BYTE1(MAG_X);
	testdatatosend[_cnt++]=BYTE0(MAG_X);
	
	testdatatosend[_cnt++]=BYTE1(MAG_Y);
	testdatatosend[_cnt++]=BYTE0(MAG_Y);
	
	testdatatosend[_cnt++]=BYTE1(MAG_Z);
	testdatatosend[_cnt++]=BYTE0(MAG_Z);
	
	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
}


void SendAttitude(float roll,float pitch,float yaw){
	u8 _cnt=0;
	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	int i;
	int32_t ALT_USE=0;
	u8 FLY_MODEL=0,ARMED=0;
	vs16 _temp;
	
	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	testdatatosend[_cnt++]=0x01;//0x01，表示本帧为姿态数据帧
	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
 
	_temp = (int)(roll*100); 
	testdatatosend[_cnt++]=BYTE1(_temp);//将要发送的数据放至发送缓冲区
	testdatatosend[_cnt++]=BYTE0(_temp);
	
	_temp = (int)(pitch*100); 
	testdatatosend[_cnt++]=BYTE1(_temp);
	testdatatosend[_cnt++]=BYTE0(_temp);

	_temp = (int)(yaw*100); 
	testdatatosend[_cnt++]=BYTE1(_temp);
	testdatatosend[_cnt++]=BYTE0(_temp);
	
	testdatatosend[_cnt++]=BYTE3(ALT_USE);//假数据，为了符合数据帧要求
	testdatatosend[_cnt++]=BYTE2(ALT_USE);
	testdatatosend[_cnt++]=BYTE1(ALT_USE);
	testdatatosend[_cnt++]=BYTE0(ALT_USE);
	
	testdatatosend[_cnt++]=FLY_MODEL;
	testdatatosend[_cnt++]=ARMED;
	
	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
}

void SendPWMIN(u8 frame,u8 *STA,u16 *OVF,u16 *VAL_UP,u16 *VAL_DOWN,u16 *PW){
	u8 _cnt=0;
	u8 sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	int i;
	
	testdatatosend[_cnt++]=0xAA;//0xAA为帧头
	testdatatosend[_cnt++]=0x05;//0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
	testdatatosend[_cnt++]=0xAF;//0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
	testdatatosend[_cnt++]=frame;//0x02，表示本帧为传感器原始数据帧
	testdatatosend[_cnt++]=0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了
 
	testdatatosend[_cnt++]=*STA;
//	testdatatosend[_cnt++]=*OVF;
	testdatatosend[_cnt++]=BYTE1(*OVF);
	testdatatosend[_cnt++]=BYTE0(*OVF);
//	testdatatosend[_cnt++]=BYTE3(*VAL_UP);
//	testdatatosend[_cnt++]=BYTE2(*VAL_UP);
	testdatatosend[_cnt++]=BYTE1(*VAL_UP);
	testdatatosend[_cnt++]=BYTE0(*VAL_UP);
//	testdatatosend[_cnt++]=BYTE3(*VAL_DOWN);
//	testdatatosend[_cnt++]=BYTE2(*VAL_DOWN);
	testdatatosend[_cnt++]=BYTE1(*VAL_DOWN);
	testdatatosend[_cnt++]=BYTE0(*VAL_DOWN);
//	testdatatosend[_cnt++]=BYTE3(*PW);
//	testdatatosend[_cnt++]=BYTE2(*PW);
	testdatatosend[_cnt++]=BYTE1(*PW);
	testdatatosend[_cnt++]=BYTE0(*PW);
	
	testdatatosend[4] = _cnt-5;//_cnt用来计算数据长度，减5为减去帧开头5个非数据字节
	
	for(i=0;i<_cnt;i++)
		sum += testdatatosend[i];
	
	testdatatosend[_cnt++]=sum;	//将sum校验数据放置最后一字节
	Usart6_Send(testdatatosend, _cnt);	//调用发送数据函数
}
