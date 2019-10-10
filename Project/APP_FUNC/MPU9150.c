#include "MPU9150.h"

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
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_PWR_MGMT_1, 0x00); //解除休眠状态,使用内部8MHz振荡器
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_SMPLRT_DIV, 0x00); //采样分频 (采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_CONFIG, 0x06);
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_INT_PIN_CFG, 0x02); //turn on Bypass Mode
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_USER_CTRL, 0x00); //close Master Mode
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_GYRO_CONFIG, 0x18); //陀螺仪满量程+-2000度/秒 (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒
    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_ACCEL_CONFIG, 0x08); //加速度满量程+-4g   (最低分辨率 = 2^15/4g = 8196LSB/g )
    I2C_ByteWrite(AK8975_I2C_ADDR, AK8975_CNTL, 0x00);
    delay_ms(100);
    I2C_ByteWrite(AK8975_I2C_ADDR, AK8975_CNTL, 0x01);

    if (I2C_ByteRead(AK8975_I2C_ADDR, AK8975_WIA) != AK8975_Device_ID) { //检查MPU6050是否正常
        return 0;
    }

    return 1;
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

    //	if(I2C_ByteRead (AK8975_I2C_ADDR,AK8975_ST1)==0x01){//判断数据状态
    //		L=I2C_ByteRead (AK8975_I2C_ADDR,REG_Address);
    //		I2C_ByteWrite(AK8975_I2C_ADDR,AK8975_CNTL,0x01);//此处非常关键，
    //    //因为日本公司的数据手册上说，在单次测量模式下，每读取一次，
    //    //会自动回归power down mode ,所以这里重新设置为单次测量模式
    //		//https://blog.csdn.net/meker1/article/details/44342161
    //		H=I2C_ByteRead (AK8975_I2C_ADDR,REG_Address+1);
    //		I2C_ByteWrite(AK8975_I2C_ADDR,AK8975_CNTL,0x01);
    //	}
    I2C_ByteWrite(AK8975_I2C_ADDR, AK8975_CNTL, 0x01);
    //	while(I2C_ByteRead (AK8975_I2C_ADDR,AK8975_ST1)==0x00);
    delay_ms(8);
    L = I2C_ByteRead(AK8975_I2C_ADDR, REG_Address);
    err = I2C_ByteRead(AK8975_I2C_ADDR, AK8975_ST2) & 0x04;
    if (err)
        L = I2C_ByteRead(AK8975_I2C_ADDR, REG_Address);
    H = I2C_ByteRead(AK8975_I2C_ADDR, REG_Address + 1);

    I2C_ByteWrite(MPU6050_SlaveAddress, MPU6050_INT_PIN_CFG, 0x00); //turn off Bypass Mode
    return (H << 8) | L; //合成
}
