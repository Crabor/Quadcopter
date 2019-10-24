#include "iic.h"

// I2C initialization. Use analog I2C
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOB时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // 开漏输出Open-drain output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// I2C start
u8 I2C_START(void)
{
    I2C_SDA_H;
    I2C_NOP;

    I2C_SCL_H;
    I2C_NOP;

    if (!I2C_SDA_STATE)
        return I2C_BUS_BUSY;

    I2C_SDA_L;
    I2C_NOP;

    I2C_SCL_L;
    I2C_NOP;

    if (I2C_SDA_STATE)
        return I2C_BUS_ERROR;

    return I2C_READY;
}

// I2C stop
void I2C_STOP(void)
{
    I2C_SDA_L;
    I2C_NOP;

    I2C_SCL_H;
    I2C_NOP;

    I2C_SDA_H;
    I2C_NOP;
}

// I2C send ACK
void I2C_SendACK(void)
{
    I2C_SDA_L;
    I2C_NOP;
    I2C_SCL_H;
    I2C_NOP;
    I2C_SCL_L;
    I2C_NOP;
}

// I2C send NACK
void I2C_SendNACK(void)
{
    I2C_SDA_H;
    I2C_NOP;
    I2C_SCL_H;
    I2C_NOP;
    I2C_SCL_L;
    I2C_NOP;
}

// I2C send one byte
u8 I2C_SendByte(u8 i2c_data)
{
    u8 i;

    I2C_SCL_L;
    for (i = 0; i < 8; i++) {
        if (i2c_data & 0x80)
            I2C_SDA_H;
        else
            I2C_SDA_L;

        i2c_data <<= 1;
        I2C_NOP;

        I2C_SCL_H;
        I2C_NOP;
        I2C_SCL_L;
        I2C_NOP;
    }

    I2C_SDA_H;
    I2C_NOP;
    I2C_SCL_H;
    I2C_NOP;
    if (I2C_SDA_STATE) {
        I2C_SCL_L;
        return I2C_NACK;
    } else {
        I2C_SCL_L;
        return I2C_ACK;
    }
}

// I2C receive one byte
u8 I2C_ReceiveByte(void)
{
    u8 i, i2c_data;

    I2C_SDA_H;
    I2C_SCL_L;
    i2c_data = 0;

    for (i = 0; i < 8; i++) {
        I2C_SCL_H;
        I2C_NOP;
        i2c_data <<= 1;

        if (I2C_SDA_STATE)
            i2c_data |= 0x01;

        I2C_SCL_L;
        I2C_NOP;
    }
    I2C_SendNACK();
    return i2c_data;
}

// I2C receive one byte with ACK
u8 I2C_ReceiveByte_WithACK(void)
{
    u8 i, i2c_data;

    I2C_SDA_H;
    I2C_SCL_L;
    i2c_data = 0;

    for (i = 0; i < 8; i++) {
        I2C_SCL_H;
        I2C_NOP;
        i2c_data <<= 1;

        if (I2C_SDA_STATE)
            i2c_data |= 0x01;

        I2C_SCL_L;
        I2C_NOP;
    }
    I2C_SendACK();
    return i2c_data;
}

// I2C写一个字节
void I2C_WriteByte(uint8_t DeviceAddr, uint8_t address, uint8_t data)
{//注意DeviceAddr为原始地址左移一位后的地址
    I2C_START();
    I2C_SendByte(DeviceAddr);
    I2C_SendByte(address);
    I2C_SendByte(data);
    I2C_STOP();
}

// I2C写一个字节（不带片内地址）
void I2C_NoAddr_WriteByte(uint8_t DeviceAddr, uint8_t data)
{//注意DeviceAddr为原始地址左移一位后的地址
    I2C_START();
    I2C_SendByte(DeviceAddr);
    I2C_SendByte(data);
    I2C_STOP();
}

// I2C读一个字节
uint8_t I2C_ReadByte(uint8_t DeviceAddr, uint8_t address)
{//注意DeviceAddr为原始地址左移一位后的地址
    uint8_t i;
    I2C_START();
    I2C_SendByte(DeviceAddr);
    I2C_SendByte(address);
    I2C_START();
    I2C_SendByte(DeviceAddr + 1);
    i = I2C_ReceiveByte();
    I2C_STOP();
    return i;
}

// I2C读两个字节
uint16_t I2C_Read_2Bytes(uint8_t DeviceAddr, uint8_t address)
{//注意DeviceAddr为原始地址左移一位后的地址
    uint8_t data_temp1, data_temp2;
    uint16_t data_16;

    I2C_START();
    I2C_SendByte(DeviceAddr);
    I2C_SendByte(address);
    I2C_START();
    I2C_SendByte(DeviceAddr + 1);
    data_temp1 = I2C_ReceiveByte_WithACK();
    data_temp2 = I2C_ReceiveByte();
    I2C_STOP();

    data_16 = (data_temp1 << 8) | data_temp2;
    return data_16;
}

// I2C读三个字节
uint32_t I2C_Read_3Bytes(uint8_t DeviceAddr, uint8_t address)
{//注意DeviceAddr为原始地址左移一位后的地址
    uint8_t data_temp1, data_temp2, data_temp3;
    uint32_t data_32;

    I2C_START();
    I2C_SendByte(DeviceAddr);
    I2C_SendByte(address);
    I2C_START();
    I2C_SendByte(DeviceAddr + 1);
    data_temp1 = I2C_ReceiveByte_WithACK();
    data_temp2 = I2C_ReceiveByte_WithACK();
    data_temp3 = I2C_ReceiveByte();
    I2C_STOP();

    data_32 = (data_temp1 << 16) | (data_temp2 << 8) | data_temp3;
    return data_32;
}

// I2C写多个字节
u8 i2cwrite(u8 dev_addr, u8 reg_addr, u8 i2c_len, u8* i2c_data_buf)
{//注意dev_addr为原始规定数据，未左移
    u8 i;
    I2C_START();
    I2C_SendByte(dev_addr << 1 | I2C_Direction_Transmitter);
    I2C_SendByte(reg_addr);
    for (i = 0; i < i2c_len; i++)
        I2C_SendByte(i2c_data_buf[i]);

    I2C_STOP();
    return 0x00;
}

// I2C读多个字节
u8 i2cread(u8 dev_addr, u8 reg_addr, u8 i2c_len, u8* i2c_data_buf)
{//注意dev_addr为原始规定数据，未左移
    I2C_START();
    I2C_SendByte(dev_addr << 1 | I2C_Direction_Transmitter);
    I2C_SendByte(reg_addr);
    I2C_START();
    I2C_SendByte(dev_addr << 1 | I2C_Direction_Receiver);

    while (i2c_len) {
        if (i2c_len == 1)
            *i2c_data_buf = I2C_ReceiveByte();
        else
            *i2c_data_buf = I2C_ReceiveByte_WithACK();
        i2c_data_buf++;
        i2c_len--;
    }
    I2C_STOP();
    return 0x00;
}
