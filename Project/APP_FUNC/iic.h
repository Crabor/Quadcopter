#ifndef __IIC_H_
#define __IIC_H_
#include "includes.h"

#define I2C_Speed 						400000

//Functions definition
void IIC_Init(void);
void I2C_ByteWrite(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t I2C_ByteRead(uint8_t SlaveAddress,uint8_t REG_Address);

#endif

