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
} Acc;

/* MPU6050--陀螺仪结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Gyro;

/* HMC5883L--磁力计结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Mag;

/* float结构体 */
typedef struct
{
    float x;
    float y;
    float z;
} Float;

/* 姿态解算--角度值 */
typedef struct
{
    float yaw;
    float roll;
    float pitch;
} Angle;

//变量定义
extern uint8_t gyroOffset, accOffset, pressOffset; //用于零偏校准
extern Acc acc, offsetAcc; //原始数据、零偏数据
extern Gyro gyro, offsetGyro; //原始数据、零偏数据
extern Mag mag; //原始数据
extern Float fGyro; //角速度数据（rad）
extern Angle angle; //姿态解算-角度值
extern float press, offsetPress; //温度补偿大气压，零偏大气压
extern float Temperature; //实际温度
extern float K_PRESS_TO_HIGH; //气压转换成高度，因为不同地区比例不一样，所以不设成宏
extern float height,velocity;//高度,速度

// 函数声明
float invSqrt(float x);
void Open_Calib(void);
u8 Calib_Status(void);
void MPU6050_Offset(void);
void GY86_Read(void);
void Quat_Init(void);
void AttitudeUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void HeightUpdate(float ax, float ay, float az,float press);
void AHRS_Time_Init(void);
float Get_AHRS_Time(void);
void applyDeadband(float *value, float deadband);

#endif
