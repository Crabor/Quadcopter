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
} PID_t;

typedef enum {
    STOP = 0, //停止
    HOVER, //悬停
    DOWN, //下降
    UP //上升
} FlyMode_t;

void PID_Init(void);
void Judge_FlyMode(float expMode);
float PID_Calc(float angleErr, float gyro, PID_t* shell, PID_t* core);
void Motor_Calc(void);
void Motor_Exp_Calc(void);
float Limit(float pwm, float min, float max);
void PID_Time_Init(void);
float Get_PID_Time(void);

#endif
