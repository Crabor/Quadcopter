#ifndef __IMU_H
#define	__IMU_H
#include "includes.h"
#include "defines.h"
#include "math.h"
#define RtA 	57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.0152672				
#define Gyro_Gr	0.0002663	   

void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ACC_ANGLE(void);
void Get_YawOffSet(void);
//float Acc_Get_Angle(float x,float y,float z,u8 dir);
extern S_FLOAT_XYZ Q_ANGLE;
extern S_FLOAT_XYZ A_ANGLE;
extern float g_fYawOffSet;
#endif


