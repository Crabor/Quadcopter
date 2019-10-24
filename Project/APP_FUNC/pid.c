#include "pid.h"

//因为结构体变量引用不能放在.h文件中，所以放这
extern Float fGyro; //角速度数据（rad）
extern Angle angle; //姿态解算-角度值

float rollShellKp = 1.0f; //外环Kp
float rollShellTi = 0.0f; //外环Ti
float rollCoreKp = 1.0f; //内环Kp
float rollCoreTi = 0.0f; //内环Ti
float rollCoreTd = 0.0f; //内环Td

float pitchShellKp = 1.0f;
float pitchShellTi = 0.0f;
float pitchCoreKp = 1.0f;
float pitchCoreTi = 0.0f;
float pitchCoreTd = 0.0f;

float yawShellKp = 1.0f;
float yawShellTi = 0.0f;
float yawCoreKp = 1.0f;
float yawCoreTi = 0.0f;
float yawCoreTd = 0.0f;

/******************************************************************************
函数原型：	void PID_Init(void)
功    能：	PID初始化
*******************************************************************************/
void PID_Init(void)
{
    rollShell.Kp = rollShellKp;
    rollShell.Ti = rollShellTi;
    rollCore.Kp = rollCoreKp;
    rollCore.Ti = rollCoreTi;
    rollCore.Td = rollCoreTd;
    pitchShell.Kp = pitchShellKp;
    pitchShell.Ti = pitchShellTi;
    pitchCore.Kp = pitchCoreKp;
    pitchCore.Ti = pitchCoreTi;
    pitchCore.Td = pitchCoreTd;
    yawShell.Kp = yawShellKp;
    yawShell.Ti = yawShellTi;
    yawCore.Kp = yawCoreKp;
    yawCore.Ti = yawCoreTi;
    yawCore.Td = yawCoreTd;
}

/******************************************************************************
函数原型：	int16_t PID_Calc(float angleErr, float gyro, PID *shell, PID *core)
功    能：	PID计算
输    入：  angleErr，角度偏差
            gyro，对应轴角速度
            shell,外环
            core，内环
返    回：  内环输出
*******************************************************************************/
int16_t PID_Calc(float angleErr, float gyro, PID *shell, PID *core)
{
    float shellOutput, coreOutput;
    float shellKi, coreKi, coreKd;

    shellKi = pidT / shell->Ti;
    coreKi = pidT / core->Ti;
    coreKd = core->Td / pidT;

    shell->eK = angleErr;
    //积分限幅
    if (shell->eSum > 300)
        shell->eSum = 300;
    else if (shell->eSum < -300)
        shell->eSum = -300;
    else
        shell->eSum += angleErr;

    shellOutput = shell->Kp * (shell->eK + shell->eSum * shellKi);

    //外环输出，作为内环输入 用陀螺仪当前的角速度作为实际值
    core->eK = shellOutput - gyro;

    //内环积分限幅
    if (core->eSum > 500)
        core->eSum = 500;
    else if (core->eSum < -500)
        core->eSum = -500;
    else
        core->eSum += core->eK;

    coreOutput = core->Kp * (core->eK + core->eSum * coreKi + (core->eK - core->eK_1) * coreKd);
    core->eK_1 = core->eK;

    return (int16_t)coreOutput;
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
    pidPitch = PID_Calc(expPitch - angle.pitch, fGyro.x, &pitchShell, &pitchCore);
    pidYaw = PID_Calc(expYaw - angle.yaw, fGyro.z, &yawShell, &yawCore);

    //PWM限幅
    motor1 = Limit_PWM(expThr - pidPitch - pidRoll - pidYaw, 1000, 2000);
    motor2 = Limit_PWM(expThr - pidPitch + pidRoll + pidYaw, 1000, 2000);
    motor3 = Limit_PWM(expThr + pidPitch - pidRoll + pidYaw, 1000, 2000);
    motor4 = Limit_PWM(expThr + pidPitch + pidRoll - pidYaw, 1000, 2000);

    //如果油门过小，则停止飞行
    if (expThr <= 1050) {
        motor1 = 1000;
        motor2 = 1000;
        motor3 = 1000;
        motor4 = 1000;
    }
}

/******************************************************************************
函数原型：	int16_t Limit_PWM(int16_t pwm, int16_t min, int16_t max)
功    能：	PWM限幅
输    入：  pwm，输入pwm值
            min，最小值
            max，最大值
返    回：  限幅后的pwm
*******************************************************************************/
int16_t Limit_PWM(int16_t pwm, int16_t min, int16_t max)
{
    return pwm < min ? min : (pwm > max ? max : pwm);
}

/******************************************************************************
函数原型：	void Motor_Exp_Calc(void)
功    能：	计算遥控器的期望值
*******************************************************************************/
void Motor_Exp_Calc(void)
{
    int16_t PWMInCh1,PWMInCh2,PWMInCh3,PWMInCh4;
    //限幅
    PWMInCh1 = Limit_PWM(PWM_IN_CH[0], 1000, 2000);
    PWMInCh2 = Limit_PWM(PWM_IN_CH[1], 1000, 2000);
    PWMInCh3 = Limit_PWM(PWM_IN_CH[2], 1000, 2000);
    PWMInCh4 = Limit_PWM(PWM_IN_CH[3], 1000, 2000);

    //转化为期望值
    expRoll = (float)((PWMInCh4 - 1500) * 0.06f); //最大倾角30°
    expPitch = (float)((PWMInCh2 - 1500) * 0.06f);
    expYaw = (float)((PWMInCh1 - 1500) * 0.06f);
    expThr = (float)PWMInCh3;
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
