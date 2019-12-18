#ifndef USART_H
#define USART_H
#include "includes.h"
//匿名上位机-https://blog.csdn.net/wangjt1988
//正点原子——《STM32F4开发指南-寄存器版本_V1.1》

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp) (*((char*)(&dwTemp)))
#define BYTE1(dwTemp) (*((char*)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp) + 3))

//函数定义
// void USART2_Init(u32 pclk2, u32 bound);
void USART6_Init(u32 pclk2, u32 bound);
void USART6_IRQ(void);
void USART6_ItSend(unsigned char* DataToSend, u8 data_num);
void USART6_NItSend(unsigned char* DataToSend, u8 data_num);

void Send_Senser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z);
void Send_Attitude(float roll, float pitch, float yaw);
void Send_RCData_Motor(int16_t THR, int16_t YAW, int16_t ROLL, int16_t PITCH, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Send_expVal(u8 frame, float expRoll, float expPitch, float expYaw, float expMode);
void Send_pidOutVal(u8 frame, float rollShellOutput, float rollCoreOutput, float pitchShellOutput, float pitchCoreOutput, float yawShellOutput, float yawCoreOutput);
void Send_Height_Temp(float height,float temp);

void SendStr(const char* str);
void SendByte(u8 frame, u8* p);
void SendHalfWord(u8 frame, u16* p);
void SendWord(u8 frame, u32* p);
//void SendPWMIN(u8 frame, u8* STA, u16* OVF, u16* VAL_UP, u16* VAL_DOWN, u16* PW);
#endif
