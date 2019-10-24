#ifndef __IIC_H_
#define __IIC_H_
#include "includes.h"

// #define I2C_Speed 		100000

// //Functions definition
// void IIC_Init(void);
// void I2C_ByteWrite(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
// uint8_t I2C_ByteRead(uint8_t SlaveAddress,uint8_t REG_Address);

#define SCL_H GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define SCL_L GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define SDA_H GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define SDA_L GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define SDA_read ((GPIOB->IDR & GPIO_Pin_7) != 0) ? 1 : 0

void IIC_Init(void); //初始化IIC的IO口
void IIC_Start(void); //发送IIC开始信号
void IIC_Stop(void); //发送IIC停止信号
void IIC_Ack(void); //IIC发送ACK信号
void IIC_NAck(void); //IIC不发送ACK信号
uint8_t IIC_WaitAck(void); //IIC等待ACK信号

void IIC_SendByte(uint8_t data); //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack); //IIC读取一个字节

uint8_t I2C_ByteRead(uint8_t I2C_Addr, uint8_t reg);
uint8_t I2C_MultByteRead(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t I2C_ByteWrite(uint8_t I2C_Addr, uint8_t reg, uint8_t buf);
uint8_t I2C_MultByteWrite(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);

#endif
