#include "IMU.h"

//变量声明
extern uint8_t gyroOffset, accOffset, pressOffset; //用于零偏校准
extern Acc_t acc, offsetAcc; //原始数据、零偏数据
extern Gyro_t gyro, offsetGyro; //原始数据、零偏数据
extern Mag_t mag; //原始数据
extern Float_t fGyro; //角速度数据（rad）
extern Angle_t angle; //姿态解算-角度值
extern float pressure, offsetPress; //温度补偿大气压，零偏大气压
extern float Temperature; //实际温度
extern float K_PRESS_TO_HIGH; //气压转换成高度，因为不同地区比例不一样，所以不设成宏
extern float height, velocity, acceleration_z; //高度（m）,垂直速度(m/s),垂直加速度（m/s^2）
extern OS_EVENT* IICMutex;

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
    pressOffset = 1;
}

/******************************************************************************
函数原型：	u8 Calib_Status(void)
功    能：	检查MPU6050零偏校正状态
返    回： 0，零偏校准完成
          1，零偏未完成
*******************************************************************************/
u8 Calib_Status(void)
{
    return accOffset | gyroOffset | pressOffset;
}

/******************************************************************************
函数原型：	void MPU6050_Offset(void)
功    能：	MPU6050零偏校正
*******************************************************************************/
void MPU6050_Offset(void)
{ //磁力计无需进行零偏校准
    if (accOffset) { //加速度计校准
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
        if (count_acc == 101) {
            count_acc--;
            offsetAcc.x = ACC_X / count_acc;
            offsetAcc.y = ACC_Y / count_acc;
            offsetAcc.z = ACC_Z / count_acc - 8192; //加速度量程±4G
            count_acc = 0;
            accOffset = 0;
        }
    }

    if (gyroOffset) { //陀螺仪校准
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
        if (count_gyro == 101) {
            count_gyro--;
            offsetGyro.x = GYRO_X / count_gyro;
            offsetGyro.y = GYRO_Y / count_gyro;
            offsetGyro.z = GYRO_Z / count_gyro;
            count_gyro = 0;
            gyroOffset = 0;
        }
    }

    if (pressOffset) { //气压计校准
        static float PRESS = 0;
        static uint8_t count_press = 0;
        if (count_press == 0) {
            offsetPress = 0;
            PRESS = 0;
            count_press = 1;
            return;
        } else {
            count_press++;
            PRESS += pressure;
        }
        if (count_press == 51) {
            count_press--;
            offsetPress = PRESS / count_press;
            count_press = 0;
            pressOffset = 0;
            //海拔-气压微分公式
            //d_High=-44300*pow(p/p_0,-4.256/5.256)*d_Pressure/(5.256*p_0)
            //d_Pressure：气压微分，单位Pa
            //p_0：标准大气压，101325Pa
            //d_High：高度微分，单位m
            //p：当地气压
            K_PRESS_TO_HIGH = -44300 * pow(offsetPress / 101325, -4.256 / 5.256) / (5.256 * 101325);
        }
    }
}

/******************************************************************************
函数原型：	void GY86_Read(Type_t type)
功    能：	读取MPU6050的16位数据
*******************************************************************************/
void GY86_Read(Type_t type)
{
    uint8_t dataBuf[14];
    INT8U err;

    if (type == ACC_GYRO_MAG) {

        //mpu6050
        OSMutexPend(IICMutex, 0, &err);
        if (!i2cread(MPU6050_Addr_Real, MPU6050_ACCEL_XOUT_H, 14, dataBuf)) {
            OSMutexPost(IICMutex);
            acc.x = ((((int16_t)dataBuf[0]) << 8) | dataBuf[1]) - offsetAcc.x;
            acc.y = ((((int16_t)dataBuf[2]) << 8) | dataBuf[3]) - offsetAcc.y;
            acc.z = ((((int16_t)dataBuf[4]) << 8) | dataBuf[5]) - offsetAcc.z;
            gyro.x = ((((int16_t)dataBuf[8]) << 8) | dataBuf[9]) - offsetGyro.x;
            gyro.y = ((((int16_t)dataBuf[10]) << 8) | dataBuf[11]) - offsetGyro.y;
            gyro.z = ((((int16_t)dataBuf[12]) << 8) | dataBuf[13]) - offsetGyro.z;

            //角速度单位转换，加速度无需转换单位
            fGyro.x = (float)(gyro.x * RAW_TO_RAD);
            fGyro.y = (float)(gyro.y * RAW_TO_RAD);
            fGyro.z = (float)(gyro.z * RAW_TO_RAD);
        }

        //HMC5883L
        OSMutexPend(IICMutex, 0, &err);
        if (!i2cread(HMC5883L_Addr_Real, HMC5883L_XOUT_MSB, 6, dataBuf)) { //必须连续读完这六个数据，否则会导致隔很长一段时间才出新数据
            OSMutexPost(IICMutex);
            mag.x = (dataBuf[0] << 8) | dataBuf[1];
            mag.y = (dataBuf[4] << 8) | dataBuf[5];
            mag.z = (dataBuf[2] << 8) | dataBuf[3];

            // 读取的原数据为补码形式，这里完成转换
            if (mag.x > 0x7fff)
                mag.x -= 0xffff;
            if (mag.y > 0x7fff)
                mag.y -= 0xffff;
            if (mag.z > 0x7fff)
                mag.z -= 0xffff;
        }
    }

    if (type == TEMP_PRESS) {
        //MS5611
        MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096); //0x58
        MS561101BA_GetPressure(MS561101BA_D1_OSR_4096); //0x48
        //pressure -= offsetPress;
    }

    MPU6050_Offset();
}

// ==================================================================================
// 描述:
// 必须定义'halfT '为周期的一半，以及滤波器的参数Kp和Ki
// 四元数'q0', 'q1', 'q2', 'q3'定义为全局变量
// 需要在每一个采样周期调用'IMUupdate()'函数
// 陀螺仪数据单位是弧度/秒，加速度计、磁力计的单位无关重要，因为会被规范化
// ==================================================================================
float Kp = 2.0f; // 比例常数
float Ki = 0.005f; // 积分常数
float halfT = 0.0005f; //采样周期的一半，实际halfT由定时器求出
float T = 0.001f; // 采样周期为1ms
float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // 四元数
float exInt = 0, eyInt = 0, ezInt = 0; // 误差积分累计值

// 四元数初始化
void Quat_Init(void)
{
    int16_t initMx, initMy, initMz;
    float initYaw, initPitch, initRoll;

    uint8_t dataBuf[14];
    if (!i2cread(HMC5883L_Addr_Real, HMC5883L_XOUT_MSB, 6, dataBuf)) {
        initMx = (dataBuf[0] << 8) | dataBuf[1];
        initMy = (dataBuf[4] << 8) | dataBuf[5];
        initMz = (dataBuf[2] << 8) | dataBuf[3];

        // Complement processing and unit conversion
        if (initMx > 0x7fff)
            initMx -= 0xffff;
        if (initMy > 0x7fff)
            initMy -= 0xffff;
        if (initMz > 0x7fff)
            initMz -= 0xffff;
    }

    //求出初始欧拉角，初始状态水平，所以roll、pitch为0
    initRoll = 0.0f;
    initPitch = 0.0f;
    initYaw = atan2(initMx * cos(initRoll) + initMy * sin(initRoll) * sin(initPitch) + initMz * sin(initRoll) * cos(initPitch),
        initMy * cos(initPitch) - initMz * sin(initPitch));

    // 四元数计算
    q0 = cos(0.5f * initRoll) * cos(0.5f * initPitch) * cos(0.5f * initYaw) + sin(0.5f * initRoll) * sin(0.5f * initPitch) * sin(0.5f * initYaw); //w
    q1 = cos(0.5f * initRoll) * sin(0.5f * initPitch) * cos(0.5f * initYaw) - sin(0.5f * initRoll) * cos(0.5f * initPitch) * sin(0.5f * initYaw); //x Pitch
    q2 = sin(0.5f * initRoll) * cos(0.5f * initPitch) * cos(0.5f * initYaw) + cos(0.5f * initRoll) * sin(0.5f * initPitch) * sin(0.5f * initYaw); //y Roll
    q3 = cos(0.5f * initRoll) * cos(0.5f * initPitch) * sin(0.5f * initYaw) - sin(0.5f * initRoll) * sin(0.5f * initPitch) * cos(0.5f * initYaw); //z Yaw
}

// ==================================================================================
// 函数原型：void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
// 功        能：互补滤波进行姿态解算
// 输        入：陀螺仪数据及加速度计数据及磁力计数据
// ==================================================================================
void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float hx, hy, hz, bz, by;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float q0_last, q1_last, q2_last, q3_last;

    //空间换时间，提高效率
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

    //加速度计归一化，这就是为什么之前只要陀螺仪数据进行单位转换
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //计算上一时刻机体坐标系下加速度坐标
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //磁力计数据归一化
    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;
    //计算地理坐标系下磁力坐标（地理南北极）
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
    //（地磁南北极）因为地磁南北极与地理南北极有偏差
    //bx=0;
    by = sqrtf((hx * hx) + (hy * hy));
    bz = hz;
    //磁力转换回机体坐标系坐标
    wx = 2 * by * (q1q2 + q0q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * by * (0.5f - q1q1 - q3q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * by * (q2q3 - q0q1) + 2 * bz * (0.5f - q1q1 - q2q2);

    //误差计算（加速度和磁场强度一起）
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    //由定时器获取采样周期的一半
    halfT = Get_AHRS_Time();

    //pi运算
    if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
        // 误差积分
        exInt += ex * Ki * halfT;
        eyInt += ey * Ki * halfT;
        ezInt += ez * Ki * halfT;

        // 角速度补偿
        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
    }

    // 保存四元数
    q0_last = q0;
    q1_last = q1;
    q2_last = q2;
    q3_last = q3;
    // 积分增量运算
    q0 = q0_last + (-q1_last * gx - q2_last * gy - q3_last * gz) * halfT;
    q1 = q1_last + (q0_last * gx + q2_last * gz - q3_last * gy) * halfT;
    q2 = q2_last + (q0_last * gy - q1_last * gz + q3_last * gx) * halfT;
    q3 = q3_last + (q0_last * gz + q1_last * gy - q2_last * gx) * halfT;

    // 归一化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm; //w
    q1 = q1 * norm; //x
    q2 = q2 * norm; //y
    q3 = q3 * norm; //z

    //四元数转欧拉角
    angle.roll = asin(2.0f * (q0 * q2 - q1 * q3)) * RAD_TO_ANGLE;
    angle.pitch = -atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * RAD_TO_ANGLE;
    angle.yaw = atan2(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD_TO_ANGLE;
}

// //死区计算
// void Apply_Deadband(float* value, float deadBand)
// {
//     if (*value<deadBand&& * value> deadBand) {
//         *value = 0;
//     } else if (value > 0) {
//         *value -= deadBand;
//     } else if (value < 0) {
//         *value += deadBand;
//     }
// }

////Combine Filter to correct err
//static void inertial_filter_predict(float dt, float x[3])
//{
//    x[0] += x[1] * dt + x[2] * dt * dt / 2.0f;
//    x[1] += x[2] * dt;
//}

//static void inertial_filter_correct(float e, float dt, float x[3], int i, float w)
//{
//    float ewdt = e * w * dt;
//    x[i] += ewdt;

//    if (i == 0) {
//        x[1] += w * ewdt;
//        x[2] += w * w * ewdt / 3.0f;

//    } else if (i == 1) {
//        x[2] += w * ewdt;
//    }
//}

//float z_est[3]; // estimate z Vz  Az
//float corr_acc[] = { 0.0f, 0.0f, 0.0f }; // 地理坐标系下加速度,  m/s2
//float acc_bias[] = { 0.0f, 0.0f, 0.0f }; // 地理坐标系下加速度偏移量 ,
//float corr_baro = 0.0f; //m

void Height_Update(/*float ax, float ay, float az, */ float pressure)
{
    //    float dT; //采样周期
    //    float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
    //    static float w_z_baro = 0.5f;
    //    static float w_z_acc = 20.0f;
    //    static float w_acc_bias = 0.05f;
    float baro_height = (pressure - offsetPress) * K_PRESS_TO_HIGH * 100; //气压计相对高度cm
    //    vs32 _temp;

    height = (height * 7 + baro_height * 3) / 10;

    //    baro_height = (baro_height*2+(pressure - offsetPress) * K_PRESS_TO_HIGH*6)/8;

    // //求采样周期
    // dT = 2 * halfT;

    // //机体加速度计减去偏移量（注意偏移量和零偏量不是一个东西）
    // ax -= acc_bias[0];
    // ay -= acc_bias[1];
    // az -= acc_bias[2];

    // //机体加速度转换到地理加速度
    // corr_acc[0] = 2 * ax * (0.5f - q2 * q2 - q3 * q3) + 2 * ay * (q1 * q2 - q0 * q3) + 2 * az * (q1 * q3 + q0 * q2);
    // corr_acc[1] = 2 * ax * (q1 * q2 + q0 * q3) + 2 * ay * (0.5f - q1 * q1 - q3 * q3) + 2 * az * (q2 * q3 - q0 * q1);
    // corr_acc[2] = 2 * ax * (q1 * q3 - q0 * q2) + 2 * ay * (q2 * q3 + q0 * q1) + 2 * az * (0.5f - q1 * q1 - q2 * q2);

    // //加速度单位转化m/s^2
    // corr_acc[0] *= 9.8f / 8192;
    // corr_acc[1] *= 9.8f / 8192;
    // corr_acc[2] *= 9.8f / 8192;
    // corr_acc[2] -= 9.8f; //地理坐标系下的z轴加速度是有重力加速度的，因此补偿上去。

    // _temp = corr_acc[0] * 100;
    // SendWord(0xF3, &_temp);
    // _temp = corr_acc[1] * 100;
    // SendWord(0xF4, &_temp);
    // _temp = corr_acc[2] * 100;
    // SendWord(0xF5, &_temp);

    // z_est[2] = (z_est[2] * 6 + corr_acc[2] * 2) / 8;
    // z_est[1] += z_est[2] * dT;
    // z_est[0] += z_est[1] * dT + 0.5f * z_est[2] * dT * dT;

    // height = (baro_height * 6 + z_est[0] * 2) / 8;

    // //计算气压计校正系数
    // corr_baro = -baro_height - z_est[0];

    // //加速度偏移向量校正
    // accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;

    // //将偏移向量转换到机体坐标系
    // ax = accel_bias_corr[2] * (2 * q1 * q3 - 2 * q0 * q2);
    // ay = accel_bias_corr[2] * (2 * q0 * q1 + 2 * q2 * q3);
    // az = accel_bias_corr[2] * (1 - 2 * q1 * q1 - 2 * q2 * q2);

    // //累加偏移量
    // acc_bias[0] += ax * w_acc_bias * dT;
    // acc_bias[1] += ay * w_acc_bias * dT;
    // acc_bias[2] += az * w_acc_bias * dT;

    // //加速度推算高度
    // inertial_filter_predict(dT, z_est);

    // //气压计校正系数进行校正
    // inertial_filter_correct(corr_baro, dT, z_est, 0, w_z_baro); //0.5f
    // inertial_filter_correct(corr_acc[2], dT, z_est, 2, w_z_acc); //20.0f

    // //求值
    // height = z_est[0];
    // velocity = z_est[1];
    // acceleration_z = z_est[2];

    //     float MS5611_press_height, MS5611_vz; //气压计高度，气压计速度
    //     float dT; //采样周期
    //     float fused_vz; //融合速度
    //     static float mpu6050_vz = 0, MS5611_press_height_old = 0, offset_az = 0; //mpu6050加速度计速度,气压计滤波后高度，mpu6050加速度滤波

    //     //机体加速度转换到地理加速度
    //     mpu6050_az = 2 * ax * (q1 * q3 - q0 * q2) + 2 * ay * (q2 * q3 + q0 * q1) + 2 * az * (0.5f - q1 * q1 - q2 * q2);

    //     //加速度单位转化cm/s^2
    //     mpu6050_az *= 9.8f / 8192;
    //     mpu6050_az -= 9.8f;
    //     mpu6050_az *= 100;

    //     //加速度低通滤波
    //     offset_az -= offset_az / 8;
    //     offset_az += mpu6050_az;
    //     mpu6050_az -= offset_az / 8;

    //     //计算气压高度cm
    //     MS5611_press_height = (pressure - offsetPress) * K_PRESS_TO_HIGH * 100;

    //     //计算高度（低通滤波）
    //     height = (height * 8 + MS5611_press_height * 2) / 10;

    //     //采样周期计算
    //     dT = 2 * halfT;

    //     //加速度计计算的速度
    //     mpu6050_vz += mpu6050_az * dT;
    //     Apply_Deadband(&mpu6050_vz, 10);
    //     //气压计计算的速度
    //     MS5611_vz = (MS5611_press_height - MS5611_press_height_old) / dT;
    //     MS5611_press_height_old = MS5611_press_height;
    //     Apply_Deadband(&MS5611_vz, 10);
    //     //计算速度（融合）
    //     fused_vz = mpu6050_vz * 0.985f + MS5611_vz * 0.015f;
    //     velocity = (velocity * 6 + fused_vz * 2) / 8;
    //     Apply_Deadband(&velocity, 5);

    // //    //高度计算补偿计算
    // //    height += velocity * dT;
}

//用于获取姿态解算采样时间
void AHRS_Time_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 关闭TIM2以进行配置
    TIM_DeInit(TIM2);
    //psc为84，arr为0xFFFF，向上计数
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 使能TIM2
    TIM_Cmd(TIM2, ENABLE);
}

// 获取halfT时间
float Get_AHRS_Time(void)
{
    float temp = 0;
    static uint32_t now = 0;

    // 获取计数器值
    now = TIM2->CNT;
    // 清空计数器值
    TIM2->CNT = 0;
    // 计数器值转换为时间并除以2
    temp = (float)now / 2000000.0f;

    return temp;
}
