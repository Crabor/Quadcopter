#ifndef __IIC_H_
#define __IIC_H_
#include "includes.h"

// I2C SCL/SDA macro definition
#define I2C_SCL            GPIO_Pin_6
#define I2C_SDA            GPIO_Pin_7

// I2C SCL/SDA set low/high macro definition
#define I2C_SCL_L          GPIO_ResetBits(GPIOB, I2C_SCL)
#define I2C_SCL_H          GPIO_SetBits(GPIOB, I2C_SCL)
#define I2C_SDA_L          GPIO_ResetBits(GPIOB, I2C_SDA)
#define I2C_SDA_H          GPIO_SetBits(GPIOB, I2C_SDA)

// I2C SDA state macro definition
#define I2C_SDA_STATE      GPIO_ReadInputDataBit(GPIOB, I2C_SDA)

// I2C state macro definition
#define I2C_READY          0x00
#define I2C_BUS_BUSY       0x01	
#define I2C_BUS_ERROR      0x02

// I2C ACK/NACK macro definition
#define I2C_NACK           0x00 
#define I2C_ACK            0x01

// I2C nop macro definition
#define I2C_NOP            delay_us(1)

// Functions definition
void IIC_Init(void);

void I2C_WriteByte(uint8_t DeviceAddr, uint8_t address, uint8_t data);
void I2C_NoAddr_WriteByte(uint8_t DeviceAddr, uint8_t data);

uint8_t I2C_ReadByte(uint8_t DeviceAddr, uint8_t address);
uint16_t I2C_Read_2Bytes(uint8_t DeviceAddr, uint8_t address);
uint32_t I2C_Read_3Bytes(uint8_t DeviceAddr, uint8_t address);

u8 i2cwrite(u8 dev_addr, u8 reg_addr, u8 i2c_len, u8 *i2c_data_buf);
u8 i2cread(u8 dev_addr, u8 reg_addr, u8 i2c_len, u8 *i2c_data_buf);

#endif
