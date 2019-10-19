#ifndef PID_H
#define PID_H
#include "includes.h"

typedef struct {
    float eK; //本次误差
    float eK_1; //上次误差
    float eSum; //误差和
    float Kp; //比例系数
    float Ti; //积分时间
    float Td; //微分时间
} PID;

extern PID rollCore, rollShell, pitchCore, pitchShell, yawCore, yawShell; //六个环的pid结构体
extern float pidT; //采样周期
extern int16_t pidRoll, pidPitch, pidYaw; //pid输出
extern float expRoll, expPitch, expYaw, expThr; //期望值
extern int16_t motor1, motor2, motor3, motor4; //四个电机速度

void PID_Init(void);
int16_t PID_Calculate(float angleErr, float gyro, PID shell, PID core);
void Motor_Calculate(void);
void Motor_Exp_Calculate(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4);
int16_t Limit_PWM(int16_t pwm, int16_t min, int16_t max);
void PID_Time_Init(void);
float Get_PID_Time(void);

#endif
