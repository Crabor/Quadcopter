#ifndef IMU_H
#define IMU_H

#include "includes.h"
#include "math.h"

#define Pi	3.1415927f
#define Radian_to_Angle	   57.2957795f
#define RawData_to_Angle	0.0610351f	//以下参数对应2000度每秒
#define RawData_to_Radian	0.0010653f
#define Filter_Num 2

/* MPU6050--加速度计结构体 */
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Acc;

/* MPU6050--陀螺仪结构体 */
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Gyro;

/* float结构体 */
typedef struct
{
	float x;
	float y;
	float z;
}Float;

/* 姿态解算--角度值 */
typedef struct
{
	float yaw;
	float roll;
	float pitch;
}Angle;

/* pid变量 */
typedef struct
{
	float kp;
	float ki;
	float kd;
	float integral;
	
	float output;
}PID;


float invSqrt(float x);
void Open_Calib(void);
void MPU6050_Offset(void);
void MPU6050_Read(void);
void Calculate_FilteringCoefficient(float Time, float cutOff);
void ACC_IIR_Filter(Acc *accIn,Acc *accOut);
void Gyro_Filter(Gyro *gyroIn,Gyro *gyroOut);
void Get_Radian(Gyro *gyroIn,Float *gyroOut);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void Get_Eulerian_Angle(Angle *angle);

#endif
