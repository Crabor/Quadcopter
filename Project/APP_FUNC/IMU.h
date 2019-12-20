#ifndef IMU_H
#define IMU_H
#include "includes.h"
#include "math.h"
//吴勇等编著——《四轴飞行器DIY——基于STM32微控制器》
//程序源码——https://pan.baidu.com/share/init?surl=o7sVC8a，密码：bpcd

#define PI 3.1415927f
//角度单位相关转换
#define RAD_TO_ANGLE 57.2957795f //弧度转角度
#define RAW_TO_ANGLE 0.0610351f //原始数据转角度，对应±2000°/s
#define RAW_TO_RAD 0.0010653f //原始数据转弧度，对应±2000°/s

#define FILTER_NUM 2

/* MPU6050--加速度计结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Acc_t;

/* MPU6050--陀螺仪结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Gyro_t;

/* HMC5883L--磁力计结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Mag_t;

/* float结构体 */
typedef struct
{
    float x;
    float y;
    float z;
} Float_t;

/* 姿态解算--角度值 */
typedef struct
{
    float yaw;
    float roll;
    float pitch;
} Angle_t;

// 函数声明
float invSqrt(float x);
void Open_Calib(void);
u8 Calib_Status(void);
void MPU6050_Offset(void);
void GY86_Read(void);
void Quat_Init(void);
void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Height_Update(float ax, float ay, float az, float pressure);
void AHRS_Time_Init(void);
float Get_AHRS_Time(void);
//void Apply_Deadband(float* value, float deadBand);

#endif
