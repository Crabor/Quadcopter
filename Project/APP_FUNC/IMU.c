#include "IMU.h"

// ==================================================================================
// 描述:
// 必须定义'halfT '为周期的一半，以及滤波器的参数Kp和Ki
// 四元数'q0', 'q1', 'q2', 'q3'定义为全局变量
// 需要在每一个采样周期调用'IMUupdate()'函数
// 陀螺仪数据单位是弧度/秒，加速度计的单位无关重要，因为会被规范化
// ==================================================================================
float Kp = 4.0f; // 比例常数
float Ki = 0.001f; // 积分常数
float halfT = 0.0005f; //采样周期的一半
float T = 0.001f; // 周期为1ms
// ==================================================================================
// 变量定义
float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // 四元数
float exInt = 0, eyInt = 0, ezInt = 0; // 误差积分累计值

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// 快速计算开根号的倒数
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/******************************************************************************
函数原型：	void Open_Calib(void)
功    能：	打开MPU6050零偏校正
*******************************************************************************/
void Open_Calib(void)
{
    accOffset = 1;
    gyroOffset = 1;
}

/******************************************************************************
函数原型：	u8 Calib_Status(void)
功    能：	检查MPU6050零偏校正状态
*******************************************************************************/
u8 Calib_Status(void)
{
    return accOffset | gyroOffset;
}

/******************************************************************************
函数原型：	void MPU9150_Offset(void)
功    能：	MPU6050零偏校正
*******************************************************************************/
void MPU9150_Offset(void)
{
    if (accOffset) {
        static int32_t ACC_X = 0, ACC_Y = 0, ACC_Z = 0;
        static uint8_t count_acc = 0;
        if (count_acc == 0) {
            offsetAcc.x = 0;
            offsetAcc.y = 0;
            offsetAcc.z = 0;
            ACC_X = 0;
            ACC_Y = 0;
            ACC_Z = 0;
            count_acc = 1;
            return;
        } else {
            count_acc++;
            ACC_X += acc.x;
            ACC_Y += acc.y;
            ACC_Z += acc.z;
        }
        if (count_acc == 251) {
            count_acc--;
            offsetAcc.x = ACC_X / count_acc;
            offsetAcc.y = ACC_Y / count_acc;
            offsetAcc.z = ACC_Z / count_acc - 8192; //加速度量程±4G
            count_acc = 0;
            accOffset = 0;
        }
    }

    if (gyroOffset) {
        static int32_t GYRO_X = 0, GYRO_Y = 0, GYRO_Z = 0;
        static uint8_t count_gyro = 0;
        if (count_gyro == 0) {
            offsetGyro.x = 0;
            offsetGyro.y = 0;
            offsetGyro.z = 0;
            GYRO_X = 0;
            GYRO_Y = 0;
            GYRO_Z = 0;
            count_gyro = 1;
            return;
        } else {
            count_gyro++;
            GYRO_X += gyro.x;
            GYRO_Y += gyro.y;
            GYRO_Z += gyro.z;
        }
        if (count_gyro == 251) {
            count_gyro--;
            offsetGyro.x = GYRO_X / count_gyro;
            offsetGyro.y = GYRO_Y / count_gyro;
            offsetGyro.z = GYRO_Z / count_gyro;
            count_gyro = 0;
            gyroOffset = 0;
        }
    }
}

/******************************************************************************
函数原型：	void MPU9150_Read(void)
功    能：	读取MPU6050的16位数据
*******************************************************************************/
void MPU9150_Read(void)
{
    acc.x = GetData_MPU6050(MPU6050_ACCEL_XOUT_H) - offsetAcc.x; //减去零偏
    acc.y = GetData_MPU6050(MPU6050_ACCEL_YOUT_H) - offsetAcc.y;
    acc.z = GetData_MPU6050(MPU6050_ACCEL_ZOUT_H) - offsetAcc.z;

    gyro.x = GetData_MPU6050(MPU6050_GYRO_XOUT_H) - offsetGyro.x;
    gyro.y = GetData_MPU6050(MPU6050_GYRO_YOUT_H) - offsetGyro.y;
    gyro.z = GetData_MPU6050(MPU6050_GYRO_ZOUT_H) - offsetGyro.z;

    mag.x = GetData_AK8975(AK8975_MAG_XOUT_L);
//	delay_ms(8);
    mag.y = GetData_AK8975(AK8975_MAG_YOUT_L)-168;
//	delay_ms(8);
    mag.z = GetData_AK8975(AK8975_MAG_ZOUT_L);
//	delay_ms(8);
//		mag.z=f(mag.z);

    MPU9150_Offset();
}

int16_t f( int16_t x) 
{
//		if ( x > -127 )
//			return (-x-127);
//		else 
//			return (-x-382);
	if(x>-127)
		return (x-127);
	else
		return (x+127);
} 

/******************************************************************************
函数原型：	void Calculate_FilteringCoefficient(float Time, float cutOff)
功    能：	iir低通滤波参数计算
*******************************************************************************/
void Calculate_FilteringCoefficient(float Time, float cutOff)
{
    ACC_IIR_FACTOR = Time / (Time + 1 / (2.0f * Pi * cutOff));
}

/******************************************************************************
函数原型：	void ACC_IIR_Filter(Acc *accIn,Acc *accOut)
功    能：	iir低通滤波
*******************************************************************************/
void ACC_IIR_Filter(Acc* accIn, Acc* accOut)
{
    accOut->x = accOut->x + ACC_IIR_FACTOR * (accIn->x - accOut->x);
    accOut->y = accOut->y + ACC_IIR_FACTOR * (accIn->y - accOut->y);
    accOut->z = accOut->z + ACC_IIR_FACTOR * (accIn->z - accOut->z);
}

/******************************************************************************
函数原型：	void Gyro_Filter(Gyro *gyroIn,Gyro *gyroOut)
功    能：	gyro窗口滑动滤波
*******************************************************************************/
void Gyro_Filter(Gyro* gyroIn, Gyro* gyroOut)
{
    static int16_t Filter_x[Filter_Num], Filter_y[Filter_Num], Filter_z[Filter_Num];
    static uint8_t Filter_count;
    int32_t Filter_sum_x = 0, Filter_sum_y = 0, Filter_sum_z = 0;
    uint8_t i = 0;

    Filter_x[Filter_count] = gyroIn->x;
    Filter_y[Filter_count] = gyroIn->y;
    Filter_z[Filter_count] = gyroIn->z;

    for (i = 0; i < Filter_Num; i++) {
        Filter_sum_x += Filter_x[i];
        Filter_sum_y += Filter_y[i];
        Filter_sum_z += Filter_z[i];
    }

    gyroOut->x = Filter_sum_x / Filter_Num;
    gyroOut->y = Filter_sum_y / Filter_Num;
    gyroOut->z = Filter_sum_z / Filter_Num;

    Filter_count++;
    if (Filter_count == Filter_Num)
        Filter_count = 0;
}

/******************************************************************************
函数原型：	void Get_Radian(Gyro *gyroIn,Float *gyroOut)
功    能：	角速度由原始数据转为弧度
*******************************************************************************/
void Get_Radian(Gyro* gyroIn, Float* gyroOut)
{
    gyroOut->x = (float)(gyroIn->x * RawData_to_Radian);
    gyroOut->y = (float)(gyroIn->y * RawData_to_Radian);
    gyroOut->z = (float)(gyroIn->z * RawData_to_Radian);
}

// ==================================================================================
// 函数原型：void IMUUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
// 功        能：互补滤波进行姿态解算
// 输        入：陀螺仪数据及加速度计数据及磁力计数据
// ==================================================================================
void IMUUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float hx, hy, hz, bz, by;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float q0_last, q1_last, q2_last;
//	u32 fmx=(int32_t)mx,fmy=(int32_t)my,fmz=(int32_t)mz;

    //auxiliary variables to reduce number of repeated operations
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    //normalise the measurements
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;
//		SendWord(0xF1,&fmx);
//		SendWord(0xF2,&fmy);
//		SendWord(0xF3,&fmz);

    //compute reference direction of flux
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);

    // bx = sqrtf((hx*hx) + (hy*hy));
    by = sqrtf((hx * hx) + (hy * hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    wx = 2 * by * (q1q2 + q0q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * by * (0.5f - q1q1 - q3q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * by * (q2q3 - q0q1) + 2 * bz * (0.5f - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    halfT = Get_AHRS_Time();

    if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
        // integral error scaled integral gain
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;

        // adjusted gyroscope measurements
        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
    }

    // save quaternion
    q0_last = q0;
    q1_last = q1;
    q2_last = q2;

    // integrate quaternion rate and normalise (Picard first order)
    q0 = q0_last + (-q1_last * gx - q2_last * gy - q3 * gz) * halfT;
    q1 = q1_last + (q0_last * gx + q2_last * gz - q3 * gy) * halfT;
    q2 = q2_last + (q0_last * gy - q1_last * gz + q3 * gx) * halfT;
    q3 = q3 + (q0_last * gz + q1_last * gy - q2_last * gx) * halfT;

    // normalise quaternion
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm; //w
    q1 = q1 * norm; //x
    q2 = q2 * norm; //y
    q3 = q3 * norm; //z

    //   angle.yaw  +=  filterGyro.z * RawData_to_Angle * 0.001f;
}

/******************************************************************************
函数原型：	void Get_Eulerian_Angle(Angle *angle)
功    能：	四元数转欧拉角
*******************************************************************************/
void Get_Eulerian_Angle(Angle* angle)
{
    angle->pitch = -atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * Radian_to_Angle;
    angle->roll = asin(2.0f * (q0 * q2 - q1 * q3)) * Radian_to_Angle;
    angle->yaw = atan2(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * Radian_to_Angle;
    //	if((fGyro.z*Radian_to_Angle>2.0f)||(fGyro.z*Radian_to_Angle<-2.0f)){
    //		angle->yaw -= fGyro.z*Radian_to_Angle*0.01f;
    //	}
}

//用于获取姿态解算采样时间
void AHRS_Time_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // Enable TIM2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Close TIM2
    TIM_DeInit(TIM2);
    // TIM2 configuration. Prescaler is 80, period is 0xFFFF, and counter mode is up
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 80 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Enable TIM2
    TIM_Cmd(TIM2, ENABLE);
}

// Get half of AHRS update time
float Get_AHRS_Time(void)
{
    float temp = 0;
    static uint32_t now = 0;

    // Get timer count
    now = TIM2->CNT;
    // Clear timer count
    TIM2->CNT = 0;
    // Convert to HZ unit and divided by 2
    temp = (float)now / 2000000.0f;

    return temp;
}
