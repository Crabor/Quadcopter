#include "pid.h"

extern PID_t rollCore, rollShell, pitchCore, pitchShell, yawCore, thrShell; //六个环的pid结构体
extern float pidT; //采样周期
extern float expRoll, expPitch, expYaw, expMode, expHeight; //期望值
extern float motor1, motor2, motor3, motor4; //四个电机速度
extern u16 PWM_IN_CH[4]; //定时器5四轴通道捕获PWM带宽值
extern FlyMode_t flyMode; //飞行模式
extern Float_t fGyro; //角速度数据（rad）
extern Angle_t angle; //姿态解算-角度值
extern float height, velocity; //高度（cm）,速度(cm/s)
extern float pidRoll, pidPitch, pidYaw, pidThr; //pid输出

float rollShellKp = 4.4f; //外环Kp
float rollCoreKp = 2.6f; //内环Kp
float rollCoreTi = 0.5f; //内环Ti
float rollCoreTd = 0.08f; //内环Td

float pitchShellKp = 4.4f;
float pitchCoreKp = 2.6f;
float pitchCoreTi = 0.5f;
float pitchCoreTd = 0.08f;

float yawCoreKp = 2.6f;
float yawCoreTd = 0.08f;

float thrShellKp = 1.0f;
float thrShellTd = 0.0f;

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
    yawCore.Td = yawCoreTd;

    //thr
    thrShell.Kp = thrShellKp;
    thrShell.Td = thrShellTd;

    //flyMode
    flyMode = STOP;

    //expHeight
    expHeight = 0;
}

/******************************************************************************
函数原型：	float PID_Calc(float shellErr, float coreStatus, PID_t *shell, PID_t *core)
功    能：	PID计算
输    入：  shellErr，角度偏差
            coreStatus，对应轴角速度
            shell,外环
            core，内环
返    回：  内环输出
*******************************************************************************/
float PID_Calc(float shellErr, float coreStatus, PID_t* shell, PID_t* core)
{
    //TODO:0是否可以当空指针
    float shellKd, coreKi, coreKd;

    //ROLL,PITCH--串级PID
    if (shell && core) {
        coreKi = pidT / core->Ti;
        coreKd = core->Td / pidT;

        shell->eK = shellErr;
        shell->output = shell->Kp * shell->eK;
        shell->eK_1 = shell->eK;

        core->eK = shell->output - coreStatus; //外环输出，作为内环输入 用陀螺仪当前的角速度作为实际值

        //TODO:内环积分浮点数比大小出问题
        //内环积分限幅
        if (core->eSum > CORE_INT_MAX) {
            if (core->eK < -0.0f)
                core->eSum += core->eK;
        } else if (core->eSum < -CORE_INT_MAX) {
            if (core->eK > 0.0f)
                core->eSum += core->eK;
        } else {
            core->eSum += core->eK;
        }

        core->output = core->Kp * (core->eK + core->eSum * coreKi + (core->eK - core->eK_1) * coreKd); //内环输出
        core->output = Limit(core->output, -PID_OUT_MAX, PID_OUT_MAX);
        core->eK_1 = core->eK;

        return core->output;
    }

    //HEIGHT--外环PID
    if (shell && !core) {
        shellKd = shell->Td / pidT;
        shell->eK = shellErr;
        shell->output = shell->Kp * (shell->eK + (shell->eK - shell->eK_1) * shellKd);
        shell->output = Limit(shell->output, -PID_OUT_MAX, PID_OUT_MAX);
        shell->eK_1 = shell->eK;
        return shell->output;
    }

    //YAW--内环PID
    if (!shell && core) {
        coreKd = core->Td / pidT;
        core->eK = expYaw - coreStatus;
        core->output = core->Kp * (core->eK + (core->eK - core->eK_1) * coreKd);
        core->output = Limit(core->output, -PID_OUT_MAX, PID_OUT_MAX);
        core->eK_1 = core->eK;
        return core->output;
    }

    return 0;
}

//飞行器模式状态机转换
void Judge_FlyMode(float expMode)
{
    switch (flyMode) {
    case STOP:
        if (expMode > 1350) {
            flyMode = HOVER;
            expHeight = 100;
        }
        break;
    case HOVER:
        if (expMode > 1650) {
            flyMode = UP;
        }
        if (expMode < 1350) {
            flyMode = DOWN;
        }
        break;
    case DOWN:
        if (expMode > 1350) {
            flyMode = HOVER;
            expHeight = height;
        }
        if (expMode < 1050) {
            flyMode = STOP;
        }
        break;
    case UP:
        if (expMode < 1650) {
            flyMode = HOVER;
            expHeight = height;
        }
        break;
    default:
        flyMode = STOP;
    }
}

/******************************************************************************
函数原型：	void Motor_Calc(void)
功    能：	计算输出速度（PWM）给四个电机
*******************************************************************************/
void Motor_Calc(void)
{
    //float pidRoll = 0, pidPitch = 0, pidYaw = 0, pidThr = 0; //pid输出
    //计算采样周期
    pidT = Get_PID_Time();

    //计算姿态PID
    //TODO:注意正负
    pidRoll = PID_Calc(expRoll - angle.roll, fGyro.y * RAD_TO_ANGLE, &rollShell, &rollCore);
    pidPitch = PID_Calc(expPitch - angle.pitch, -fGyro.x * RAD_TO_ANGLE, &pitchShell, &pitchCore);
    //TODO:yaw 与pitch、roll的pid计算不一样
    //pidYaw = PID_Calc(0,  fGyro.z * RAD_TO_ANGLE, 0, &yawCore);

    //PWM限幅
    motor1 = Limit(expMode - pidPitch + pidRoll - pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    motor2 = Limit(expMode - pidPitch - pidRoll + pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    motor3 = Limit(expMode + pidPitch + pidRoll + pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    motor4 = Limit(expMode + pidPitch - pidRoll - pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);

    //如果机体处于停止模式或倾斜角大于65度，则停止飞行
    if (expMode <= 1050 || angle.pitch >= 65 || angle.pitch <= -65 || angle.roll >= 65 || angle.roll <= -65) {
        motor1 = PWM_OUT_MIN;
        motor2 = PWM_OUT_MIN;
        motor3 = PWM_OUT_MIN;
        motor4 = PWM_OUT_MIN;
    }

    // //飞行模式判断
    // Judge_FlyMode(expMode);

    // if (flyMode == HOVER) {
    //     pidThr = PID_Calc(expHeight - height, 0, &thrShell, 0);
    // } else if (flyMode == UP) {
    //     pidThr = PID_Calc((expMode - 1650) * 0.1f, 0, &thrShell, 0);
    // } else if (flyMode == DOWN) {
    //     pidThr = PID_Calc((expMode - 1350) * 0.1f, 0, &thrShell, 0);
    // }

    // //PWM限幅
    // //TODO:1500是否是起飞临界值
    // motor1 = Limit(1500 + pidThr - pidPitch + pidRoll - pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    // motor2 = Limit(1500 + pidThr - pidPitch - pidRoll + pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    // motor3 = Limit(1500 + pidThr + pidPitch + pidRoll + pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);
    // motor4 = Limit(1500 + pidThr + pidPitch - pidRoll - pidYaw, PWM_OUT_MIN, PWM_OUT_MAX);

    // //如果机体处于停止模式或倾斜角大于65度，则停止飞行
    // if (flyMode == STOP || angle.pitch >= 65 || angle.pitch <= -65 || angle.roll >= 65 || angle.roll <= -65) {
    //     motor1 = PWM_OUT_MIN;
    //     motor2 = PWM_OUT_MIN;
    //     motor3 = PWM_OUT_MIN;
    //     motor4 = PWM_OUT_MIN;
    // }
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
    expRoll = (float)((PWMInCh4 - 1500) * 0.03f); //最大20度
    expPitch = (float)((PWMInCh2 - 1500) * 0.04f); //最大20度
    //TODO:yaw与roll、pitch不一样
    expYaw = (float)((PWMInCh1 - 1500) * 0.02f); //最大10度每秒
    expMode = PWMInCh3; //模式
}

//用于求pid采样时间
void PID_Time_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // Enable TIM4 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Close TIM4
    TIM_DeInit(TIM4);
    // TIM4 configuration. Prescaler is 84, period is 0xFFFF, and counter mode is up
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
