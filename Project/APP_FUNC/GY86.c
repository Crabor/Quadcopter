#include "GY86.h"
//因为MPU9150里面的磁力计AK8975有问题，所以后面改MPU9150为GY86了，里面的磁力计HMC5883L性能较好。
//两种外设间切换通过AK8975_EN这个宏决定。为了避免麻烦所以文件名还是统一用MPU9150

uint16_t Cal_C[7]; //用于存放PROM中的6组数据
uint32_t D1_Pres, D2_Temp; // 存放数字压力和温度
extern float Pressure; //温度补偿大气压
extern float Temperature; //实际温度
float dT, Temperature2; //实际和参考温度之间的差异,中间值
double OFF, SENS; //实际温度抵消,实际温度灵敏度
float Aux, OFF2, SENS2; //温度校验值
uint32_t ex_Pressure; //串口读数转换值
uint8_t exchange_num[8];

/**************************************
 * 函数名：MPU6050_Init
 * 描述  ：初始化Mpu6050
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
int MPU6050_Init(void)
{
    if (I2C_ReadByte(MPU6050_Addr, MPU6050_WHO_AM_I) != MPU6050_Device_ID) { //检查MPU6050是否正常
        return 0;
    }
    I2C_WriteByte(MPU6050_Addr, MPU6050_PWR_MGMT_1, 0x00); //解除休眠状态,使用内部8MHz振荡器
    I2C_WriteByte(MPU6050_Addr, MPU6050_SMPLRT_DIV, 0x00); //采样分频 (采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
    I2C_WriteByte(MPU6050_Addr, MPU6050_CONFIG, 0x06); //设置低通滤波
    I2C_WriteByte(MPU6050_Addr, MPU6050_GYRO_CONFIG, 0x18); //陀螺仪满量程+-2000度/秒 (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒
    I2C_WriteByte(MPU6050_Addr, MPU6050_ACCEL_CONFIG, 0x08); //加速度满量程+-4g   (最低分辨率 = 2^15/4g = 8192LSB/g )
    I2C_WriteByte(MPU6050_Addr, MPU6050_INT_PIN_CFG, 0x02);//打开旁路模式
    I2C_WriteByte(MPU6050_Addr, MPU6050_USER_CTRL, 0x00);//关闭主模式

    return 1;
}

/**************************************
 * HMC5883L_Init
 * 描述  ：初始化HMC5883L
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void HMC5883L_Init(void)
{
    // 设置标准数据输出速率75HZ
    I2C_WriteByte(HMC5883L_Addr, HMC5883L_CONFIG_A, 0x18);
    // 设置传感器磁场范围±1.3Ga
    I2C_WriteByte(HMC5883L_Addr, HMC5883L_CONFIG_B, 0x20);
    // 打开continuous measurement模式
    I2C_WriteByte(HMC5883L_Addr, HMC5883L_MODE, 0x00);
}

/************************************************************   
* 函数名:MS561101BA_Init   
* 描述 : MS561101BA初始化
* 输入  :无   
* 输出  :无    
*/
void MS561101BA_Init(void)
{
    MS561101BA_Reset();
    delay_ms(100);
    MS561101BA_readPROM();
    delay_ms(100);
}

/*************************************
 * 函数名：GetData_MPU6050
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ************************************/
uint16_t GetData_MPU6050(uint8_t REG_Address)
{ //不再使用，因为连续读六个数据更好
    uint8_t H, L;
    H = I2C_ReadByte(MPU6050_Addr, REG_Address);
    L = I2C_ReadByte(MPU6050_Addr, REG_Address + 1);
    return (H << 8) | L; //合成数据
}

/*************************************
 * 函数名：GetData_HMC5883L
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ************************************/
uint16_t GetData_HMC5883L(uint8_t REG_Address)
{ //不再使用，因为连续读六个数据更好
    uint8_t H, L;
    H = I2C_ReadByte(HMC5883L_Addr, REG_Address);
    L = I2C_ReadByte(HMC5883L_Addr, REG_Address + 1);
    return (H << 8) | L; //合成数据
}

/************************************************************   
* 函数名:MS561101BA_Reset   
* 描述 : 复位  
* 输入  :无   
* 输出  :无    
*/
void MS561101BA_Reset(void)
{
    I2C_NoAddr_WriteByte(MS561101BA_Addr, MS561101BA_RESET);
}

/************************************************************   
* 函数名:MS561101BA_readPROM   
* 描述 : 从PROM读取出厂校准数据
* 输入  :无   
* 输出  :无    
*/
void MS561101BA_readPROM(void)
{
    uint16_t value = 0;
    u8 temp1[2] = { 0 };
    u8 i;
    for (i = 0; i <= MS561101BA_PROM_REG_COUNT; i++) {
        // I2C_Read_MultiBytes(MS561101BA_Addr,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2,temp1);

        //value=temp1[0]<<8|temp1[1];
        //Cal_C[i]=value;
        Cal_C[i] = I2C_Read_2Bytes(MS561101BA_Addr, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    }
}

/************************************************************   
* 函数名:MS561101BA_DO_CONVERSION   
* 描述 :  
* 输入  :无   
* 输出  :无    
*/
uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
    uint32_t conversion;

    I2C_NoAddr_WriteByte(MS561101BA_Addr, command);

    switch (command & 0x0f) { //延时,去掉数据错误
    case 0:
        delay_us(900);
        break;
    case 2:
        delay_ms(6);
        break;
    case 4:
        delay_ms(8);
        break;
    case 6:
        delay_ms(12);
        break;
    case 8:
        delay_ms(20);
        break;
    }

    conversion = I2C_Read_3Bytes(MS561101BA_Addr, 0);

    return conversion;
}

/************************************************************   
* 函数名:MS561101BA_GetTemperature   
* 描述 : 读取数字温度
* 输入  :过采样率   
* 输出  :无    
*/
void MS561101BA_GetTemperature(u8 OSR_Temp)
{

    D2_Temp = MS561101BA_DO_CONVERSION(OSR_Temp);
    delay_ms(10);

    dT = D2_Temp - (((uint32_t)Cal_C[5]) << 8);
    Temperature = 2000 + dT * ((uint32_t)Cal_C[6]) / 0x800000; //算出温度值的100倍，2001表示20.01°
}

/************************************************************   
* 函数名:MS561101BA_GetPressure   
* 描述 : 读取数字气压
* 输入  :过采样率   
* 输出  :无    
*/
void MS561101BA_GetPressure(u8 OSR_Pres)
{

    D1_Pres = MS561101BA_DO_CONVERSION(OSR_Pres);

    delay_ms(10);

    OFF = (uint32_t)(Cal_C[2] << 16) + ((uint32_t)Cal_C[4] * dT) / 0x80;
    SENS = (uint32_t)(Cal_C[1] << 15) + ((uint32_t)Cal_C[3] * dT) / 0x100;
    //温度补偿
    if (Temperature < 2000) // second order temperature compensation when under 20 degrees C
    {
        Temperature2 = (dT * dT) / 0x80000000;
        Aux = (Temperature - 2000) * (Temperature - 2000);
        OFF2 = 2.5f * Aux;
        SENS2 = 1.25f * Aux;
        if (Temperature < -1500) {
            Aux = (Temperature + 1500) * (Temperature + 1500);
            OFF2 = OFF2 + 7 * Aux;
            SENS2 = SENS + 5.5f * Aux;
        }
    } else //(Temperature > 2000)
    {
        Temperature2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }

    Temperature = Temperature - Temperature2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    Pressure = (D1_Pres * SENS / 0x200000 - OFF) / 0x8000;
}
