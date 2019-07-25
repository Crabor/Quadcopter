#ifndef __IIC_Init_H_
#define __IIC_Init_H_
#include "includes.h"

#define I2C_Speed 						400000
void I2C_Congiguration(void);
void I2C_ByteWrite(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
void I2C_ByteWrite_Ms5611(uint8_t SlaveAddress,uint8_t Commands);
uint8_t I2C_ByteRead(uint8_t SlaveAddress,uint8_t REG_Address);
uint16_t I2C_ByteRead_Ms5611_16BIT(uint8_t REG_Address);
uint32_t I2C_ByteRead_Ms5611_24BIT(uint8_t REG_Address);

#endif

