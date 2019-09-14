#ifndef __GY86_H_
#define __GY86_H_
#include "includes.h"
#include "math.h"

/* MPU6050 Register Address ------------------------------------------------------------*/
#define	SMPLRT_DIV						0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU6050_CONFIG				0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG						0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG					0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H					0x3B
#define	ACCEL_XOUT_L					0x3C
#define	ACCEL_YOUT_H					0x3D
#define	ACCEL_YOUT_L					0x3E
#define	ACCEL_ZOUT_H					0x3F
#define	ACCEL_ZOUT_L					0x40
#define	TEMP_OUT_H						0x41
#define	TEMP_OUT_L						0x42
#define	GYRO_XOUT_H						0x43
#define	GYRO_XOUT_L						0x44	
#define	GYRO_YOUT_H						0x45
#define	GYRO_YOUT_L						0x46
#define	GYRO_ZOUT_H						0x47
#define	GYRO_ZOUT_L						0x48
#define	PWR_MGMT_1						0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I							0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU6050_SlaveAddress	0xD0	//IIC写入时的地址字节数据

/* HMC5883L Register Address ------------------------------------------------------------*/

#define	HMC5883L_CONFIG_A					0x00
#define	HMC5883L_CONFIG_B					0x01
#define	HMC5883L_MODE							0x02
#define	HMC5883L_X_MSB						0x03
#define	HMC5883L_X_LSB						0x04
#define	HMC5883L_Z_MSB						0x05
#define	HMC5883L_Z_LSB						0x06
#define	HMC5883L_Y_MSB						0x07
#define	HMC5883L_Y_LSB						0x08
#define	HMC5883L_STATUS						0x09
#define	HMC5883L_IDEN_REG_A				0x10
#define	HMC5883L_IDEN_REG_B				0x11
#define	HMC5883L_IDEN_REG_C				0x12
#define HMC5883L_READ_ADDRESS     0x3d
#define HMC5883L_WRITE_ADDRESS    0x3c

#define MPU6050_INT_PIN_CFG       0x37//第二位设为1
#define MPU6050_I2C_BYPASS_EN     0x02

/* Ms5611 Register Address ------------------------------------------------------------*/
#define MS561101BA_ADC_RD          0x00
#define	MS561101BA_PROM_RD 	       0xA0
#define MS561101BA_PROM_CRC        0xAE

#define MS561101BA_SlaveAddress    0xEE  //MS5611的地址
#define MS561101BA_RST             0x1E  //cmd 复位


#define MS5611_OSR256					 		 0x40
#define MS5611_OSR512					 		 0x42
#define MS5611_OSR1024					   0x44
#define MS5611_OSR2048					   0x46
#define MS5611_OSR4096					   0x48
//---------------------------------------------------------------------------------------//
//---------------------------------------变量区------------------------------------------//
typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}S_INT16_XYZ;//类型定义


extern S_INT16_XYZ	 MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//最新一次读取值,因为hmc5883l的数据可以通过MPU6050读取；
extern S_INT16_XYZ	 GYRO_OFFSET,ACC_OFFSET;//定义加速度与角速度零偏
extern S_INT16_XYZ	 ACC_AVG;

extern S_INT16_XYZ HMC5883L_MAGN_LAST;

extern float H;
extern float P1,P2;
//---------------------------------------------------------------------------------------//


/*--------函数声明---------*/
int MPU6050_Init(void); 
uint16_t GetData_MPU6050(uint8_t REG_Address);
uint8_t Read_Filter_MPU6050(void);
void Cal_Offset_MPU6050(void);
//---------------------------------------------------//
void 	HMC5883L_Init(void);
uint16_t GetData_HMC5883L(uint16_t REG_Address);
float Read_HMC5883L(void);
//---------------------------------------------------//
void MS5611_Init(void);
uint32_t MS5611_Read24bits(uint8_t command);
float MS5611_GetTemperature(uint8_t OSR_Temp);
float MS5611_GetPressure(uint8_t OSR_Pres);
float Get_High(uint8_t OSR);//OSR为压强转换速率
//---------------------------------------------------//



#endif

