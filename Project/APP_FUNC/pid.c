#include "pid.h"

//因为结构体变量引用不能放在.h文件中，所以放这
extern Float fGyro; //角速度数据（rad）
extern Angle angle; //姿态解算-角度值

float rollShellKp = 1.6f; //外环Kp
float rollCoreKp = 4.0f; //内环Kp
float rollCoreTi = 0.08f; //内环Ti
float rollCoreTd = 0.05f; //内环Td

float pitchShellKp = 1.0f;
float pitchCoreKp = 1.0f;
float pitchCoreTi = 10000.0f;
float pitchCoreTd = 0.0f;

float yawCoreKp = 1.0f;
float yawCoreTi = 10000.0f;
float yawCoreTd = 0.0f;

/******************************************************************************
函数原型：	void PID_Init(void)
功    能：	PID初始化
*******************************************************************************/
void PID_Init(void)
{
    //roll
    rollShell.Kp = rollShellKp;
    rollCore.Kp = rollCoreKp;
    rollCore.Ti = rollCoreTi;
    rollCore.Td = rollCoreTd;

    //pitch
    pitchShell.Kp = pitchShellKp;
    pitchCore.Kp = pitchCoreKp;
    pitchCore.Ti = pitchCoreTi;
    pitchCore.Td = pitchCoreTd;

    //yaw
    yawCore.Kp = yawCoreKp;
    yawCore.Ti = yawCoreTi;
    yawCore.Td = yawCoreTd;
}

/******************************************************************************
函数原型：	float PID_Calc(float angleErr, float gyro, PID *shell, PID *core)
功    能：	PID计算
输    入：  angleErr，角度偏差
            gyro，对应轴角速度
            shell,外环
            core，内环
返    回：  内环输出
*******************************************************************************/
float PID_Calc(float angleErr, float gyro, PID* shell, PID* core)
{
    float shellKd, coreKi, coreKd;
    int coreKiFlag;

    shellKd = shell->Td / pidT;
    coreKi = pidT / core->Ti;
    coreKd = core->Td / pidT;

    // shell->eK = angleErr;
    // shell->output = shell->Kp * shell->eK;

    // core->eK = shell->output - gyro*RAD_TO_ANGLE; //外环输出，作为内环输入 用陀螺仪当前的角速度作为实际值
    core->eK = expRoll - gyro * RAD_TO_ANGLE; //角速度一定要化成角度/秒，弧度/秒害死人

    // //内环积分分离
    // if (core->eK > CORE_INT_SEP_MAX || core->eK < CORE_INT_SEP_MIN) {
    //     coreKiFlag = 0;
    // } else {
    //     coreKiFlag = 1;
    //     //内环积分限幅
    //     if (core->eSum > CORE_INT_MAX) {
    //         if (core->eK < 0.0f)
    //             core->eSum += core->eK;
    //     } else if (core->eSum < CORE_INT_MIN) {
    //         if (core->eK > 0.0f)
    //             core->eSum += core->eK;
    //     } else {
    //         core->eSum += core->eK;
    //     }
    // }

    //TODO:内环积分浮点数比大小出问题
    //内环积分限幅
    if (core->eSum > CORE_INT_MAX) {
        if (core->eK < -3.0f)
            core->eSum += core->eK;
    } else if (core->eSum < CORE_INT_MIN) {
        if (core->eK > 3.0f)
            core->eSum += core->eK;
    } else {
        core->eSum += core->eK;
    }

    coreKiFlag = 0;

    core->output = core->Kp * (core->eK + core->eSum * coreKi * coreKiFlag + (core->eK - core->eK_1) * coreKd); //内环输出
    core->output = Limit(core->output, -PID_OUT_MAX, PID_OUT_MAX);
    core->eK_1 = core->eK;

    return core->output;
}

/******************************************************************************
函数原型：	void Motor_Calc(void)
功    能：	计算输出速度（PWM）给四个电机
*******************************************************************************/
void Motor_Calc(void)
{
    //计算采样周期
    pidT = Get_PID_Time();

    //计算PID
    //TODO:注意正负
    pidRoll = PID_Calc(expRoll - angle.roll, fGyro.y, &rollShell, &rollCore);
    //pidPitch =  PID_Calc(expPitch - angle.pitch, -fGyro.x, &pitchShell, &pitchCore); //
    //TODO:yaw 与pitch、roll的pid计算不一样
    //pidYaw = PID_Calc(expYaw - angle.yaw, fGyro.z, &yawShell, &yawCore);

    //PWM限幅
    motor1 = Limit(expThr - pidPitch + pidRoll - pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    motor2 = Limit(expThr - pidPitch - pidRoll + pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    motor3 = Limit(expThr + pidPitch + pidRoll + pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    motor4 = Limit(expThr + pidPitch - pidRoll - pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);

    //如果油门过小或者机体倾斜角大于65度，则停止飞行
    if (expThr <= 1050 || angle.pitch >= 65 || angle.pitch <= -65 || angle.roll >= 65 || angle.roll <= -65) {
        motor1 = PWM_OUT_MIN;
        motor2 = PWM_OUT_MIN;
        motor3 = PWM_OUT_MIN;
        motor4 = PWM_OUT_MIN;
    }
}

/******************************************************************************
函数原型：	float Limit(float pwm, float min, float max)
功    能：	PWM限幅
输    入：  pwm，输入pwm值
            min，最小值
            max，最大值
返    回：  限幅后的pwm
*******************************************************************************/
float Limit(float pwm, float min, float max)
{
    return pwm < min ? min : (pwm > max ? max : pwm);
}

/******************************************************************************
函数原型：	void Motor_Exp_Calc(void)
功    能：	计算遥控器的期望值
*******************************************************************************/
void Motor_Exp_Calc(void)
{
    int16_t PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4;
    //限幅
    PWMInCh1 = Limit(PWM_IN_CH[0], 1000, 2000);
    PWMInCh2 = Limit(PWM_IN_CH[1], 1000, 2000);
    PWMInCh3 = Limit(PWM_IN_CH[2], 1000, 2000);
    PWMInCh4 = Limit(PWM_IN_CH[3], 1000, 2000);

    //调节角速度环
    //    expRoll = (PWMInCh4 - 1500)*0.1f;

    //转化为期望值
    expRoll = ((PWMInCh4 - 1500) * 0.04f); //最大倾角20°
    expPitch = ((PWMInCh2 - 1500) * 0.04f);
    //TODO:yaw与roll、pitch不一样
    // expYaw = (float)((PWMInCh1 - 1500) * 0.04f);
    expThr = PWMInCh3;
}

//用于求pid采样时间
void PID_Time_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // Enable TIM4 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Close TIM4
    TIM_DeInit(TIM4);
    // TIM4 configuration. Prescaler is 80, period is 0xFFFF, and counter mode is up
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 80 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // Enable TIM4
    TIM_Cmd(TIM4, ENABLE);
}

// Get PID update time
float Get_PID_Time(void)
{
    float temp = 0;
    static uint32_t now = 0;

    // Get timer count
    now = TIM4->CNT;
    // Clear timer count
    TIM4->CNT = 0;
    // Convert to HZ unit
    temp = (float)now / 1000000.0f;

    return temp;
}
