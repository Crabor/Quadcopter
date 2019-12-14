#ifndef PID_H
#define PID_H
#include "includes.h"

#define CORE_INT_SEP_MAX 300.0f //内环积分分离幅值 
#define CORE_INT_MAX 4000.0f //内环积分幅值
#define PID_OUT_MAX 500.0f //PID输出幅值
#define PWM_OUT_MAX 2000.0f //PWM输出幅值最大值
#define PWM_OUT_MIN 1000.0f //PWM输出幅值最小值

typedef struct {
    float eK; //本次误差
    float eK_1; //上次误差
    float eSum; //误差和
    float Kp; //比例系数
    float Ti; //积分时间
    float Td; //微分时间
    float output; //pid输出
} PID;

extern PID rollCore, rollShell, pitchCore, pitchShell, yawCore; //五个环的pid结构体
extern float pidT; //采样周期
extern float pidRoll, pidPitch, pidYaw; //pid输出
extern float expRoll, expPitch, expYaw, expThr; //期望值
extern float motor1, motor2, motor3, motor4; //四个电机速度
extern u16 PWM_IN_CH[4]; //定时器5四轴通道捕获PWM带宽值

void PID_Init(void);
float PID_Calc(float angleErr, float gyro, PID* shell, PID* core);
void Motor_Calc(void);
void Motor_Exp_Calc(void);
float Limit(float pwm, float min, float max);
void PID_Time_Init(void);
float Get_PID_Time(void);

#endif
