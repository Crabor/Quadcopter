#ifndef IMU_H
#define IMU_H
#include "includes.h"
#include "math.h"
//吴勇等编著——《四轴飞行器DIY——基于STM32微控制器》
//程序源码——https://pan.baidu.com/share/init?surl=o7sVC8a，密码：bpcd

#define Pi	3.1415927f
//角度单位相关转换
#define Radian_to_Angle 57.2957795f
#define RawData_to_Angle 0.0610351f //以下参数对应±2000°/s
#define RawData_to_Radian 0.0010653f
//磁场强度单位相关转换
#define RawData_to_Gauss 0.003f //对应±12Gs
#define Filter_Num 2

//Struct typedef
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

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Mag;


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

//Variable declaration
extern uint8_t	gyroOffset;//不自动校正，用于零偏校准
extern uint8_t	accOffset;
extern Acc acc,filterAcc,offsetAcc;//原始数据、滤波后数据、零偏数据
extern Gyro gyro,filterGyro,offsetGyro;//原始数据、滤波后数据、零偏数据
extern Mag mag;//原始数据
extern Float fAcc,fGyro,fMag;//加速度数据（m/s2）、角速度数据（rad）、磁场强度数据（Gs）
extern Angle angle;//姿态解算-角度值
extern PID pitch,roll,gyroPitch,gyroRoll,gyroYaw;
extern float ACC_IIR_FACTOR;

// Functions definition
float invSqrt(float x);
void Open_Calib(void);
u8 Calib_Status(void);
void MPU9150_Offset(void);
void MPU9150_Read(void);
void Calculate_FilteringCoefficient(float Time, float cutOff);
void ACC_IIR_Filter(Acc *accIn,Acc *accOut);
void Gyro_Filter(Gyro *gyroIn,Gyro *gyroOut);
void Get_Radian(Gyro *gyroIn,Float *gyroOut);
void Get_Gauss(Mag *magIn,Float *magOut);
void IMUUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Get_Eulerian_Angle(Angle *angle);

#endif
