#include "pid.h"

//因为结构体变量引用不能放在.h文件中，所以放这
extern Float fGyro; //角速度数据（rad）
extern Angle angle; //姿态解算-角度值

float rollShellKp = 1.0f; //外环Kp
float rollShellTi = 1.0f; //外环Ti
float rollCoreKp = 1.0f; //内环Kp
float rollCoreTi = 1.0f; //内环Ti
float rollCoreTd = 0.0f; //内环Td

float pitchShellKp = 1.0f;
float pitchShellTi = 1.0f;
float pitchCoreKp = 1.0f;
float pitchCoreTi = 1.0f;
float pitchCoreTd = 0.0f;

float yawShellKp = 1.0f;
float yawShellTi = 1.0f;
float yawCoreKp = 1.0f;
float yawCoreTi = 1.0f;
float yawCoreTd = 0.0f;

/******************************************************************************
函数原型：	void PID_Init(void)
功    能：	PID初始化
*******************************************************************************/
void PID_Init(void)
{
    //roll
    rollShell.Kp = rollShellKp;
    rollShell.Ti = rollShellTi;
    rollShell.eSum = 0.0f;
    rollShell.output = 0.0f;
    rollCore.Kp = rollCoreKp;
    rollCore.Ti = rollCoreTi;
    rollCore.Td = rollCoreTd;
    rollCore.eK = 0.0f;
    rollCore.eK_1 = 0.0f;
    rollCore.eSum = 0.0f;
    rollCore.output = 0.0f;

    //pitch
    pitchShell.Kp = pitchShellKp;
    pitchShell.Ti = pitchShellTi;
    pitchShell.eSum = 0.0f;
    pitchShell.output = 0.0f;
    pitchCore.Kp = pitchCoreKp;
    pitchCore.Ti = pitchCoreTi;
    pitchCore.Td = pitchCoreTd;
    pitchCore.eK = 0.0f;
    pitchCore.eK_1 = 0.0f;
    pitchCore.eSum = 0.0f;
    pitchCore.output = 0.0f;

    //yaw
    yawShell.Kp = yawShellKp;
    yawShell.Ti = yawShellTi;
    yawShell.eSum = 0.0f;
    yawShell.output = 0.0f;
    yawCore.Kp = yawCoreKp;
    yawCore.Ti = yawCoreTi;
    yawCore.Td = yawCoreTd;
    yawCore.eK = 0.0f;
    yawCore.eK_1 = 0.0f;
    yawCore.eSum = 0.0f;
    yawCore.output = 0.0f;
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
    float shellKi, coreKi, coreKd;

    shellKi = pidT / shell->Ti;
    coreKi = pidT / core->Ti;
    coreKd = core->Td / pidT;

    shell->eK = angleErr;
    shell->eSum = Limit(shell->eSum + angleErr, -300, 300); //外环积分限幅
    shell->output = shell->Kp * (shell->eK + shell->eSum * shellKi); //外环输出

    core->eK = shell->output - gyro; //外环输出，作为内环输入 用陀螺仪当前的角速度作为实际值
    core->eSum = Limit(core->eSum + core->eK, -500, 500); //内环积分限幅
    core->output = core->Kp * (core->eK + core->eSum * coreKi + (core->eK - core->eK_1) * coreKd); //内环输出

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
    pidPitch = PID_Calc(expPitch - angle.pitch, -fGyro.x, &pitchShell, &pitchCore); //
    //TODO:yaw 与pitch、roll的pid计算不一样
    //pidYaw = PID_Calc(expYaw - angle.yaw, fGyro.z, &yawShell, &yawCore);

    motor1 = 1000;
    motor2 = 1000;
    motor3 = 1000;
    motor4 = 1000;

    //PWM限幅
    motor1 = Limit(expThr - pidPitch + pidRoll - pidYaw, 1000, 2000);
    motor2 = Limit(expThr - pidPitch - pidRoll + pidYaw, 1000, 2000);
    motor3 = Limit(expThr + pidPitch + pidRoll + pidYaw, 1000, 2000);
    motor4 = Limit(expThr + pidPitch - pidRoll - pidYaw, 1000, 2000);

    //如果油门过小或者机体过于倾斜，则停止飞行
    if (expThr <= 1050 || angle.pitch >= 35 || angle.pitch <= -35 || angle.roll >= 35 || angle.roll <= -35) {
        motor1 = 1000;
        motor2 = 1000;
        motor3 = 1000;
        motor4 = 1000;
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
