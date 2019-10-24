#include "iic.h"

/******************************************************************************
* 函  数：void IIC_Init(void)
* 功　能：模拟IIC引脚初始化
* 参  数：无
* 返回值：无
* 备  注：PB6->SCL	PB7->SDA
*******************************************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    // Enable GPIOB clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // GPIO configuration, Open-drain output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/******************************************************************************
* 函  数：void SDA_OUT(void)
* 功　能：配置模拟IIC SDA引脚为输出
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/******************************************************************************
* 函  数：void SDA_IN(void)
* 功　能：配置模拟IIC SDA引脚为输入
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/******************************************************************************
* 函  数：void IIC_Start(void)
* 功　能：产生IIC起始信号
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void IIC_Start(void)
{
    SDA_OUT(); //SDA线输出
    SDA_H;
    SCL_H;
    delay_us(4);
    SDA_L;
    delay_us(4);
    SCL_L;
}

/******************************************************************************
* 函  数：void IIC_Stop(void)
* 功　能：产生IIC停止信号
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void IIC_Stop(void)
{
    SDA_OUT(); //SDA线输出
    SCL_L;
    SDA_L;
    delay_us(4);
    SCL_H;
    SDA_H;
    delay_us(4);
}

/******************************************************************************
* 函  数: uint8_t IIC_WaitAck(void)
* 功　能: 等待应答信号到来 （有效应答：从机第9个 SCL=0 时 SDA 被从机拉低,
*         并且 SCL = 1时 SDA依然为低）
* 参  数：无
* 返回值：1，接收应答失败	0，接收应答成功
* 备  注：从机给主机的应答
*******************************************************************************/
uint8_t IIC_WaitAck(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN(); //SDA设置为输入  （从机给一个低电平做为应答）
    SDA_H;
    delay_us(1);
    SCL_H;
    delay_us(1);
    ;
    while (SDA_read) {
        ucErrTime++;
        if (ucErrTime > 250) {
            IIC_Stop();
            return 1;
        }
    }
    SCL_L; //时钟输出低
    return 0;
}

/******************************************************************************
* 函  数: void IIC_Ack(void)
* 功　能: 产生ACK应答 （主机接收完一个字节数据后，主机产生的ACK通知从机一个
*         字节数据已正确接收）
* 参  数：无
* 返回值：无
* 备  注：主机给从机的应答
*******************************************************************************/
void IIC_Ack(void)
{
    SCL_L;
    SDA_OUT();
    SDA_L;
    delay_us(1);
    SCL_H;
    delay_us(2);
    SCL_L;
}

/******************************************************************************
* 函  数: void IIC_NAck(void)
* 功　能: 产生NACK应答 （主机接收完最后一个字节数据后，主机产生的NACK通知从机
*         发送结束，释放SDA,以便主机产生停止信号）
* 参  数：无
* 返回值：无
* 备  注：主机给从机的应答
*******************************************************************************/
void IIC_NAck(void)
{
    SCL_L;
    SDA_OUT();
    SDA_H;
    delay_us(1);
    SCL_H;
    delay_us(1);
    SCL_L;
}

/******************************************************************************
* 函  数：void IIC_SendByte(uint8_t data)
* 功  能：IIC发送一个字节
* 参  数：data 要写的数据
* 返回值：无
* 备  注：主机往从机发
*******************************************************************************/
void IIC_SendByte(uint8_t data)
{
    uint8_t t;
    SDA_OUT();
    SCL_L; //拉低时钟开始数据传输
    for (t = 0; t < 8; t++) {
        if (data & 0x80)
            SDA_H;
        else
            SDA_L;
        data <<= 1;
        delay_us(1);
        SCL_H;
        delay_us(1);
        SCL_L;
        delay_us(1);
    }
}

/******************************************************************************
* 函  数：uint8_t IIC_ReadByte(uint8_t ack)
* 功  能：IIC读取一个字节
* 参  数：ack=1 时，主机数据还没接收完 ack=0 时主机数据已全部接收完成
* 返回值：无
* 备  注：从机往主机发
*******************************************************************************/
uint8_t IIC_ReadByte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN(); //SDA设置为输入模式 等待接收从机返回数据
    for (i = 0; i < 8; i++) {
        SCL_L;
        delay_us(1);
        SCL_H;
        receive <<= 1;
        if (SDA_read)
            receive++; //从机发送的电平
        delay_us(1);
    }
    if (ack)
        IIC_Ack(); //发送ACK
    else
        IIC_NAck(); //发送nACK
    return receive;
}

/******************************************************************************
* 函  数：uint8_t I2C_ByteRead(uint8_t I2C_Addr,uint8_t addr)
* 功　能：读取指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          *buf      读取数据要存储的地址    
* 返回值：返回 1失败 0成功
* 备  注：无
*******************************************************************************/
uint8_t I2C_ByteRead(uint8_t I2C_Addr, uint8_t reg)
{
    uint8_t ret;
    IIC_Start();
    IIC_SendByte(I2C_Addr); //发送从机地址
    if (IIC_WaitAck()) //如果从机未应答则数据发送失败
    {
        IIC_Stop();
        return 1;
    }
    IIC_SendByte(reg); //发送寄存器地址
    IIC_WaitAck();

    IIC_Start();
    IIC_SendByte(I2C_Addr + 1); //进入接收模式
    IIC_WaitAck();
    ret = IIC_ReadByte(0);
    IIC_Stop(); //产生一个停止条件
    return ret;
}

/*************************************************************************************
* 函  数：uint8_t I2C_ByteWrite(uint8_t I2C_Addr,uint8_t addr，uint8_t buf))
* 功　能：写入指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          buf       要写入的数据
* 返回值：1 失败 0成功
* 备  注：无
**************************************************************************************/
uint8_t I2C_ByteWrite(uint8_t I2C_Addr, uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_SendByte(I2C_Addr); //发送从机地址
    if (IIC_WaitAck()) {
        IIC_Stop();
        return 1; //从机地址写入失败
    }
    IIC_SendByte(reg); //发送寄存器地址
    IIC_WaitAck();
    IIC_SendByte(data);
    if (IIC_WaitAck()) {
        IIC_Stop();
        return 1; //数据写入失败
    }
    IIC_Stop(); //产生一个停止条件
    return 0;
}

/***************************************************************************************
* 函  数：uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
* 功　能：读取指定设备 指定寄存器的 length个值
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
          length  要读的字节数
		  *data   读出的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
***************************************************************************************/
uint8_t I2C_MultByteRead(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
    uint8_t count = 0;
    uint8_t temp;
    IIC_Start();
    IIC_SendByte(dev); //发送从机地址
    if (IIC_WaitAck()) {
        IIC_Stop();
        return 1; //从机地址写入失败
    }
    IIC_SendByte(reg); //发送寄存器地址
    IIC_WaitAck();
    IIC_Start();
    IIC_SendByte(dev + 1); //进入接收模式
    IIC_WaitAck();
    for (count = 0; count < length; count++) {
        if (count != (length - 1))
            temp = IIC_ReadByte(1); //带ACK的读数据
        else
            temp = IIC_ReadByte(0); //最后一个字节NACK

        data[count] = temp;
    }
    IIC_Stop(); //产生一个停止条件
    return 0;
}

/****************************************************************************************
* 函  数：uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
* 功　能：将多个字节写入指定设备 指定寄存器
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
		  length  要写的字节数
		  *data   要写入的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
****************************************************************************************/
uint8_t I2C_MultByteWrite(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
    uint8_t count = 0;
    IIC_Start();
    IIC_SendByte(dev); //发送从机地址
    if (IIC_WaitAck()) {
        IIC_Stop();
        return 1; //从机地址写入失败
    }
    IIC_SendByte(reg); //发送寄存器地址
    IIC_WaitAck();
    for (count = 0; count < length; count++) {
        IIC_SendByte(data[count]);
        if (IIC_WaitAck()) //每一个字节都要等从机应答
        {
            IIC_Stop();
            return 1; //数据写入失败
        }
    }
    IIC_Stop(); //产生一个停止条件
    return 0;
}

// void IIC_Init(void)
// {
// GPIO_InitTypeDef GPIO_InitStructure;
// I2C_InitTypeDef I2C_InitStructure;

// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOB, ENABLE);

// RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

// GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
// GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);

// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIO_Pin_6 | GPIO_Pin_7
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

// GPIO_Init(GPIOB, &GPIO_InitStructure);

// /* I2C 配置 */
// I2C_DeInit(I2C1);
// I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
// // I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
// // I2C_InitStructure.I2C_OwnAddress1 = 0xd0;
// I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
// I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
// I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;

// /* 使能 I2C1 */
// I2C_Cmd(I2C1, ENABLE);

// /* I2C1 初始化 */
// I2C_Init(I2C1, &I2C_InitStructure);

// /*允许1字节1应答模式*/
// I2C_AcknowledgeConfig(I2C1, ENABLE);
// }

// /******************************************
//  * 函数名：I2C_ByteWrite
//  * 描述  ：写一个字节到I2C设备寄存器中
//  * 输入  ：SlaveAddress写入数据器件地址,REG_Address 接收数据的IIC设备寄存器的地址
//  *         REG_data 待写入的数据
//  * 输出  ：无
//  * 返回  ：无
//  * 调用  ：内部调用
//  ******************************************/
// void I2C_ByteWrite(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
// {

//     I2C_GenerateSTART(I2C1,ENABLE);

//     while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

//     I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Transmitter);

//     while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

//     I2C_SendData(I2C1,REG_Address);

//     while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//     I2C_SendData(I2C1,REG_data);

//     while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//     I2C_GenerateSTOP(I2C1,ENABLE);
//     // int dummy;

//     // /* Send STRAT condition */

//     // I2C_GenerateSTART(I2C1, ENABLE);

//     // /* Test on EV5 and clear it */

//     // while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
//     //     ;

//     // /* Send EEPROM address for write */

//     // dummy = I2C1->SR1;

//     // I2C_Send7bitAddress(I2C1, SlaveAddress, I2C_Direction_Transmitter);

//     // /* Test on EV6 and clear it */

//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//     //     ;

//     // /* Send the EEPROM's internal address to write to */

//     // I2C_SendData(I2C1, REG_Address);

//     // /* Test on EV8 and clear it */

//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//     //     ;

//     // /* Send the byte to be written */

//     // I2C_SendData(I2C1, REG_data);

//     // /* Test on EV8 and clear it */

//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//     //     ;

//     // /* Send STOP condition */

//     // I2C_GenerateSTOP(I2C1, ENABLE);
// }

// /***********************************************
//  * 函数名：I2C_ByteRead
//  * 描述  ：从IIC设备寄存器中读取一个字节
//  * 输入  ：SlaveAddress写入数据器件地址,REG_Address 读取数据的寄存器的地址
//  * 输出  ：无
//  * 返回  ：无
//  * 调用  ：内部调用
// ************************************************/
// uint8_t I2C_ByteRead(uint8_t SlaveAddress, uint8_t REG_Address)
// {
//     uint8_t REG_data;
//     while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
//         ;
//     I2C_GenerateSTART(I2C1, ENABLE); //起始信号
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
//         ;
//     I2C_Send7bitAddress(I2C1, SlaveAddress, I2C_Direction_Transmitter); //发送设备地址+写信号
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//         ; //
//     I2C_Cmd(I2C1, ENABLE);
//     I2C_SendData(I2C1, REG_Address); //发送存储单元地址，从0开始
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//         ;
//     I2C_GenerateSTART(I2C1, ENABLE); //起始信号
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
//         ;
//     I2C_Send7bitAddress(I2C1, SlaveAddress, I2C_Direction_Receiver); //发送设备地址+读信号
//     while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//         ;
//     I2C_AcknowledgeConfig(I2C1, DISABLE);
//     I2C_GenerateSTOP(I2C1, ENABLE);
//     while (!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)))
//         ;
//     REG_data = I2C_ReceiveData(I2C1); //读出寄存器数据

//     return REG_data;
//     // uint8_t data;
//     // I2C_GenerateSTART(I2C1, ENABLE);
//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
//     //     ;
//     // I2C_AcknowledgeConfig(I2C1, DISABLE);
//     // I2C_Send7bitAddress(I2C1, SlaveAddress, I2C_Direction_Transmitter);
//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//     //     ;
//     // I2C_SendData(I2C1, REG_Address);
//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//     //     ;
//     // I2C_GenerateSTART(I2C1, ENABLE);
//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
//     //     ;
//     // I2C_Send7bitAddress(I2C1, SlaveAddress, I2C_Direction_Receiver);
//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//     //     ;
//     // I2C_GenerateSTOP(I2C1, ENABLE);
//     // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
//     //     ;
//     // data = I2C_ReceiveData(I2C1);

//     // return data;
// }
