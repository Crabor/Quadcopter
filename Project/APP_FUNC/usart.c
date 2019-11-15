#include "usart.h"

//初始化IO 串口6
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
void USART6_Init(u32 pclk2, u32 bound)
{ //PC6,PC7
    //注意，如果要使用串口中断发送，则“TC中断”不能在初始化里面使能，
    //必须要在发送完第一个数据后再使能，否则丢失第一个字节
    //因为在串口发送使能时STM32会自动发送一个空闲帧导致SR寄存器的TC位置1
    float temp;
    u16 mantissa;
    u16 fraction;
    temp = (float)(pclk2 * 1000000) / (bound * 16); //得到USARTDIV@OVER8=0
    mantissa = temp; //得到整数部分
    fraction = (temp - mantissa) * 16; //得到小数部分@OVER8=0
    mantissa <<= 4;
    mantissa += fraction;
    RCC->AHB1ENR |= 1 << 2; //使能PORTC口时钟
    RCC->APB2ENR |= 1 << 5; //使能串口6时钟
    GPIO_Set(GPIOC, GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_PuPd_UP); //PC6,PC7,复用功能,上拉输出
    GPIO_AF_Set(GPIOC, 6, 8); //PC6,AF8
    GPIO_AF_Set(GPIOC, 7, 8); //PC7,AF8
    //波特率设置
    USART6->BRR = mantissa; //波特率设置
    USART6->CR1 &= ~(1 << 15); //设置OVER8=0
    USART6->CR1 |= 1 << 3; //串口发送使能
//使能接收中断
//    USART6->CR1|=1<<2;    //串口接收使能
//    USART6->CR1|=1<<5;      //接收缓冲区非空中断使能
#if USART_IT_EN
    MY_NVIC_Init(2, 0, USART6_IRQn, 2); //组2,抢占优先级2，响应优先级0
#endif
    USART6->CR1 |= 1 << 13; //串口使能
}

/****************************************串口中断发送方式**********************************************/
u8 FLAG_TC = 0; //定义全局变量

void USART6_IRQ(void)
{
    if (USART_GetITStatus(USART6, USART_IT_TC) != RESET) //发送完成中断,= SET
    {
        USART_ClearITPendingBit(USART6, USART_IT_TC);
        FLAG_TC = 1;
    }
}

//DataToSend:发送数组
//data_num:数组长度
void USART6_ItSend(u8* DataToSend, u8 data_num)
{
    u8 i = 0;
    FLAG_TC = 0; //提前准备一下
    for (i = 0; i < data_num; i++) //检测字符串结束符
    {
        USART_SendData(USART6, *(DataToSend + i)); //发送当前字符
        if (!(USART6->CR1 & USART_CR1_TCIE)) //判断USART6->CR1中的TCIE位是否设置，即“发送完成中断”是否使能
        {
            USART_ITConfig(USART6, USART_IT_TC, ENABLE);
        }
        while (FLAG_TC == 0)
            ; //0：发送还未完成；1：发送完成
        FLAG_TC = 0;
    }
}
/*********************https://blog.csdn.net/qq_35629563/article/details/80879819***********************/

/****************************************串口非中断发送方式**********************************************/
void USART6_NItSend(unsigned char* DataToSend, u8 data_num)
{
    int i;
    for (i = 0; i < data_num; i++) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
            ; //初始化终串口发送使能时STM32会自动发送一个空闲帧，导致TC位置1
        USART_SendData(USART6, DataToSend[i]);
    }
}
/******************************************************************************************************/

void SendStr(const char* str)
{
    u8 _cnt = 0;
    u8 i = 0;
    u8 sum = 0;

    sendBuf[_cnt++] = 0xAA;
    sendBuf[_cnt++] = 0x05;
    sendBuf[_cnt++] = 0xAF;
    sendBuf[_cnt++] = 0xA0;
    sendBuf[_cnt++] = 0;
    while (*(str + i) != '\0') {
        sendBuf[_cnt++] = *(str + i);
        i++;
        if (_cnt > 50)
            break;
    }

    sendBuf[4] = _cnt - 5;

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum;

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void SendByte(u8 frame, u8* p)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = *p; //将要发送的数据放至发送缓冲区

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void SendHalfWord(u8 frame, u16* p)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(*p); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void SendWord(u8 frame, u32* p)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE3(*p); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE2(*p);
    sendBuf[_cnt++] = BYTE1(*p);
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void Send_Senser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z) //发送用户数据，这里有6个数据
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x02; //0x02，表示本帧为传感器原始数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(ACCEL_X); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(ACCEL_X);

    sendBuf[_cnt++] = BYTE1(ACCEL_Y);
    sendBuf[_cnt++] = BYTE0(ACCEL_Y);

    sendBuf[_cnt++] = BYTE1(ACCEL_Z);
    sendBuf[_cnt++] = BYTE0(ACCEL_Z);

    sendBuf[_cnt++] = BYTE1(GYRO_X);
    sendBuf[_cnt++] = BYTE0(GYRO_X);

    sendBuf[_cnt++] = BYTE1(GYRO_Y);
    sendBuf[_cnt++] = BYTE0(GYRO_Y);

    sendBuf[_cnt++] = BYTE1(GYRO_Z);
    sendBuf[_cnt++] = BYTE0(GYRO_Z);

    sendBuf[_cnt++] = BYTE1(MAG_X);
    sendBuf[_cnt++] = BYTE0(MAG_X);

    sendBuf[_cnt++] = BYTE1(MAG_Y);
    sendBuf[_cnt++] = BYTE0(MAG_Y);

    sendBuf[_cnt++] = BYTE1(MAG_Z);
    sendBuf[_cnt++] = BYTE0(MAG_Z);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void Send_Attitude(float roll, float pitch, float yaw)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int32_t ALT_USE = 0;
    u8 FLY_MODEL = 0, ARMED = 0;
    vs16 _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x01; //0x01，表示本帧为姿态数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    _temp = (int)(roll * 100);
    sendBuf[_cnt++] = BYTE1(_temp); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(pitch * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(yaw * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[_cnt++] = BYTE3(ALT_USE); //假数据，为了符合数据帧要求
    sendBuf[_cnt++] = BYTE2(ALT_USE);
    sendBuf[_cnt++] = BYTE1(ALT_USE);
    sendBuf[_cnt++] = BYTE0(ALT_USE);

    sendBuf[_cnt++] = FLY_MODEL;
    sendBuf[_cnt++] = ARMED;

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void Send_RCData_Motor(int16_t THR, int16_t YAW, int16_t ROLL, int16_t PITCH, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int16_t AUX = 0;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x03; //0x03，表示本帧为接收机、电机速度数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(THR); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(THR);

    sendBuf[_cnt++] = BYTE1(YAW);
    sendBuf[_cnt++] = BYTE0(YAW);

    sendBuf[_cnt++] = BYTE1(ROLL);
    sendBuf[_cnt++] = BYTE0(ROLL);

    sendBuf[_cnt++] = BYTE1(PITCH);
    sendBuf[_cnt++] = BYTE0(PITCH);

    sendBuf[_cnt++] = BYTE1(motor1);
    sendBuf[_cnt++] = BYTE0(motor1);

    sendBuf[_cnt++] = BYTE1(motor2);
    sendBuf[_cnt++] = BYTE0(motor2);

    sendBuf[_cnt++] = BYTE1(motor3);
    sendBuf[_cnt++] = BYTE0(motor3);

    sendBuf[_cnt++] = BYTE1(motor4);
    sendBuf[_cnt++] = BYTE0(motor4);

    sendBuf[_cnt++] = BYTE1(AUX);
    sendBuf[_cnt++] = BYTE0(AUX);

    sendBuf[_cnt++] = BYTE1(AUX);
    sendBuf[_cnt++] = BYTE0(AUX);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void Send_expVal(u8 frame, float expRoll, float expPitch, float expYaw, float expThr)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int16_t _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    _temp = expRoll;
    sendBuf[_cnt++] = BYTE1(_temp); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = expPitch;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = expYaw;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = expThr;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

void Send_pidOutVal(u8 frame, float rollShellOutput, float rollCoreOutput, float pitchShellOutput, float pitchCoreOutput, float yawShellOutput, float yawCoreOutput)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int16_t _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    _temp = rollShellOutput;
    sendBuf[_cnt++] = BYTE1(_temp); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = rollCoreOutput;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = pitchShellOutput;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = pitchCoreOutput;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = yawShellOutput;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = yawCoreOutput;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

#if USART_IT_EN
    USART6_ItSend(sendBuf, _cnt);
#else
    USART6_NItSend(sendBuf, _cnt);
#endif
}

// void SendPWMIN(u8 frame, u8* STA, u16* OVF, u16* VAL_UP, u16* VAL_DOWN, u16* PW)
// {
//     u8 _cnt = 0;
//     u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
//     int i;

//     sendBuf[_cnt++] = 0xAA; //0xAA为帧头
//     sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
//     sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
//     sendBuf[_cnt++] = frame; //用户自定义数据帧
//     sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

//     sendBuf[_cnt++] = *STA;
//     sendBuf[_cnt++] = BYTE1(*OVF);
//     sendBuf[_cnt++] = BYTE0(*OVF);
//     sendBuf[_cnt++] = BYTE1(*VAL_UP);
//     sendBuf[_cnt++] = BYTE0(*VAL_UP);
//     sendBuf[_cnt++] = BYTE1(*VAL_DOWN);
//     sendBuf[_cnt++] = BYTE0(*VAL_DOWN);
//     sendBuf[_cnt++] = BYTE1(*PW);
//     sendBuf[_cnt++] = BYTE0(*PW);

//     sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

//     for (i = 0; i < _cnt; i++)
//         sum += sendBuf[i];

//     sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

// #if USART_IT_EN
//     USART6_ItSend(sendBuf, _cnt);
// #else
//     USART6_NItSend(sendBuf, _cnt);
// #endif
// }
