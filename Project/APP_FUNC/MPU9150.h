#ifndef __MPU9150_H_
#define __MPU9150_H_
#include "includes.h"
#include "math.h"

/*Gyroscope Features(MPU6050)
The triple-axis MEMS gyroscope in the MPU-9150 includes a wide range of features:
        Digital-output X-, Y-, and Z-Axis angular rate sensors (gyroscopes) with a user-programmable full-scale range of ±250, ±500, ±1000, and ±2000°/sec
        External sync signal connected to the FSYNC pin supports image, video and GPS synchronization
        Integrated 16-bit ADCs enable simultaneous sampling of gyros
        Enhanced bias and sensitivity temperature stability reduces the need for user calibration
        Improved low-frequency noise performance
        Digitally-programmable low-pass filter
        Factory calibrated sensitivity scale factor
        User self-test

Accelerometer Features(MPU6050)
The triple-axis MEMS accelerometer in MPU-9150 includes a wide range of features:
        Digital-output 3-Axis accelerometer with a programmable full scale range of ±2g, ±4g, ±8g and ±16g
        Integrated 16-bit ADCs enable simultaneous sampling of accelerometers while requiring no external multiplexer
        Orientation detection and signaling
        Tap detection
        User-programmable interrupts
        High-G interrupt
        User self-test

Magnetometer Features(AK8975 in MPU-9150)
The triple-axis MEMS magnetometer in MPU-9150 includes a wide range of features:
        3-axis silicon monolithic Hall-effect magnetic sensor with magnetic concentrator
        Wide dynamic measurement range and high resolution with lower current consumption.
        Output data resolution is 13 bit (0.3 ?T per LSB)
        Full scale measurement range is ±1200 ?T
        Self-test function with internal magnetic source to confirm magnetic sensor operation on end products

Magnetometer Features(HMC5883L in GY-86)
        3-Axis Magnetoresistive Sensors and ASIC in a 3.0x3.0x0.9mm LCC Surface Mount Package
        Small Size for Highly Integrated Products. Just Add a Micro 
                Controller Interface, Plus Two External SMT Capacitors
                Designed for High Volume, Cost Sensitive OEM Designs
                Easy to Assemble & Compatible with High Speed SMT Assembly
        12-Bit ADC Coupled with Low Noise
                AMR Sensors Achieves 5 milli-gauss
                Resolution in ±8 Gauss Fields
        Enables 1° to 2° Degree Compass Heading Accuracy
        Built-In Self Test         
        Enables Low-Cost Functionality Test after Assembly in Production
        Low Voltage Operations (2.16 to 3.6V) and Low Power Consumption (100 μA)         
        Compatible for Battery Powered Applications
        Built-In Strap Drive Circuits         
        Set/Reset and Offset Strap Drivers for Degaussing, Self Test, and Offset Compensation
        I2C Digital Interface         
        Popular Two-Wire Serial Data Interface for Consumer Electronics
        Lead Free Package Construction         
        RoHS Compliance
        Wide Magnetic Field Range (+/-8 Oe)         
        Sensors Can Be Used in Strong Magnetic Field Environments with a 1° to 2° Degree Compass Heading Accuracy
        Software and Algorithm Support Available         
        Compassing Heading, Hard Iron, Soft Iron, and Auto Calibration Libraries Available
        Fast 160 Hz Maximum Output Rate         
        Enables Pedestrian Navigation and LBS Applications
*/

/* MPU6050 Register Address ------------------------------------------------------------*/
#define MPU6050_Addr 0xD0 //IIC写入时的地址字节数据，0x68左移一位，I2C_ReadByte()、I2C_WriteByte()时用
#define MPU6050_Addr_Real 0x68 //i2cread()、i2cwrite()时用这个地址
#define MPU6050_Device_ID 0x68

#define MPU6050_SMPLRT_DIV 0x19 //陀螺仪采样率，典型值：0x07(125Hz)
#define MPU6050_CONFIG 0x1A //低通滤波频率，典型值：0x06(5Hz)
#define MPU6050_GYRO_CONFIG 0x1B //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPU6050_ACCEL_CONFIG 0x1C //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B //电源管理，典型值：0x00(正常启用)
#define MPU6050_WHO_AM_I 0x75 //IIC地址寄存器(默认数值0x68，只读)

/* AK8975 Register Address ------------------------------------------------------------*/
#define AK8975_Addr 0x18 //IIC写入时的地址字节数据，0x0C左移一位，I2C_ReadByte()、I2C_WriteByte()时用
#define AK8975_Addr_Real 0x0C //i2cread()、i2cwrite()时用这个地址
#define AK8975_Device_ID 0x48

#define AK8975_WIA 0x00
#define AK8975_INFO 0x01
#define AK8975_ST1 0x02
#define AK8975_MAG_XOUT_L 0x03
#define AK8975_MAG_XOUT_H 0x04
#define AK8975_MAG_YOUT_L 0x05
#define AK8975_MAG_YOUT_H 0x06
#define AK8975_MAG_ZOUT_L 0x07
#define AK8975_MAG_ZOUT_H 0x08
#define AK8975_ST2 0x09
#define AK8975_CNTL 0x0A
#define AK8975_ASTC 0x0C
#define AK8975_I2CDIS 0x0F
#define AK8975_ASAX 0x10
#define AK8975_ASAY 0x11
#define AK8975_ASAZ 0x12

/* HMC5883L Register Address ------------------------------------------------------------*/
#define HMC5883L_Addr 0x3C //IIC写入时的地址字节数据，0x1E左移一位，I2C_ReadByte()、I2C_WriteByte()时用
#define HMC5883L_Addr_Real 0x1E //i2cread()、i2cwrite()时用这个地址

#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_XOUT_MSB 0x03
#define HMC5883L_XOUT_LSB 0x04
#define HMC5883L_ZOUT_MSB 0x05
#define HMC5883L_ZOUT_LSB 0x06
#define HMC5883L_YOUT_MSB 0x07
#define HMC5883L_YOUT_LSB 0x08
#define HMC5883L_StatusRegister 0x09
#define HMC5883L_ID_A 0x0A
#define HMC5883L_ID_B 0x0B
#define HMC5883L_ID_C 0x0C

//函数定义
int MPU6050_Init(void);
void HMC5883L_Init(void);
uint16_t GetData_MPU6050(uint8_t REG_Address);
uint16_t GetData_HMC5883L(uint8_t REG_Address);
uint16_t GetData_AK8975(uint8_t REG_Address);

#endif
