#include "MPU9150.h"
//因为MPU9150里面的磁力计AK8975有问题，所以后面改MPU9150为GY86了，里面的磁力计HMC5883L性能较好。
//两种外设间切换通过AK8975_EN这个宏决定。为了避免麻烦所以文件名还是统一用MPU9150



/**************************************
 * 函数名：MPU6050_Init
 * 描述  ：初始化Mpu6050
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
int MPU6050_Init(void)
{
    if (I2C_ByteRead(MPU6050_SlaveAddress, MPU6050_WHO_AM_I) != MPU6050_Device_ID) { //检查MPU6050是否正常
        return 0;
    }
//     I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_PWR_MGMT_1, 0x00); //解除休眠状态,使用内部8MHz振荡器
//     I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_SMPLRT_DIV, 0x00); //采样分频 (采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
//     I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_CONFIG, 0x06);
//     I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_INT_PIN_CFG, 0x02); //turn on Bypass Mode
//     I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_USER_CTRL, 0x00); //close Master Mode
//     I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_GYRO_CONFIG, 0x18); //陀螺仪满量程+-2000度/秒 (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒
//     I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_ACCEL_CONFIG, 0x08); //加速度满量程+-4g   (最低分辨率 = 2^15/4g = 8196LSB/g )

// #if AK8975_EN
//     I2C_ByteWrite(AK8975_I2C_ADDR, AK8975_CNTL, 0x00);
//     delay_ms(100);
//     I2C_ByteWrite(AK8975_I2C_ADDR, AK8975_CNTL, 0x01);

//     if (I2C_ByteRead(AK8975_I2C_ADDR, AK8975_WIA) != AK8975_Device_ID) { //检查MPU6050是否正常
//         return 0;
//     }
// #endif

    return 1;
}

// HMC5883L initialization
void HMC5883L_Init(void)
{
    // Set the standard data output rate to 75HZ
    I2C_ByteWrite(HMC5883L_Addr, HMC5883L_ConfigurationRegisterA, 0x18);
    // Set the sampling frequency to +-1.3Ga
    I2C_ByteWrite(HMC5883L_Addr, HMC5883L_ConfigurationRegisterB, 0x20);
    // Turn on continuous measurement mode
    I2C_ByteWrite(HMC5883L_Addr, HMC5883L_ModeRegister, 0x00);
}


/*************************************
 * 函数名：GetData_MPU6050
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ************************************/
uint16_t GetData_MPU6050(uint8_t REG_Address)
{
    uint8_t H, L;
    H = I2C_ByteRead(MPU6050_SlaveAddress, REG_Address);
    L = I2C_ByteRead(MPU6050_SlaveAddress, REG_Address + 1);
    return (H << 8) | L; //合成数据
}

/*************************************
 * 函数名：GetData_HMC5883L
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ************************************/
uint16_t GetData_HMC5883L(uint8_t REG_Address){
    uint8_t H, L;
    H = I2C_ByteRead(HMC5883L_Addr, REG_Address);
    L = I2C_ByteRead(HMC5883L_Addr, REG_Address + 1);
    return (H << 8) | L; //合成数据
}

/*************************************
 * 函数名：GetData_AK8975_MAG
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ************************************/
uint16_t GetData_AK8975(uint8_t REG_Address)
{
    uint8_t H, L, err;
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_INT_PIN_CFG, 0x02); //turn on Bypass Mode
    
    I2C_ByteWrite(AK8975_I2C_ADDR, AK8975_CNTL, 0x01);
    //	while(I2C_ByteRead (AK8975_I2C_ADDR,AK8975_ST1)==0x00);
    delay_ms(10);
    L = I2C_ByteRead(AK8975_I2C_ADDR, REG_Address);
    err = I2C_ByteRead(AK8975_I2C_ADDR, AK8975_ST2) & 0x04;
    if (err)
        L = I2C_ByteRead(AK8975_I2C_ADDR, REG_Address);
    H = I2C_ByteRead(AK8975_I2C_ADDR, REG_Address + 1);

    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_INT_PIN_CFG, 0x00); //turn off Bypass Mode
    return (H << 8) | L; //合成
}
