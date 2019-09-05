#include "GY86.h"
#define MPU6050_FILTER_NUM 10
#define MSS5611_FILTER_NUM 20
S_INT16_XYZ	 MPU6050_ACC_LAST,MPU6050_GYRO_LAST;//最新一次读取值,因为hmc5883l的数据可以通过MPU6050读取；
S_INT16_XYZ	 GYRO_OFFSET,ACC_OFFSET;//定义加速度与角速度零偏
S_INT16_XYZ	 ACC_AVG;


/**************************************
 * 函数名：MPU6050_Init
 * 描述  ：初始化Mpu6050
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void MPU6050_Init(void)
{
	I2C_ByteWrite(MPU6050_SlaveAddress,PWR_MGMT_1,0x00);//解除休眠状态
	OSTimeDly(20);	
	I2C_ByteWrite(MPU6050_SlaveAddress,SMPLRT_DIV,0x07);
	OSTimeDly(20);	
	I2C_ByteWrite(MPU6050_SlaveAddress,MPU6050_CONFIG,0x06);
	OSTimeDly(20);	
	I2C_ByteWrite(MPU6050_SlaveAddress,GYRO_CONFIG,0x08);//+-500度/秒
	OSTimeDly(20);	
	I2C_ByteWrite(MPU6050_SlaveAddress,ACCEL_CONFIG,0x08);//+-4G

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
	uint8_t H,L;
	H=I2C_ByteRead(MPU6050_SlaveAddress,REG_Address);
	L=I2C_ByteRead(MPU6050_SlaveAddress,REG_Address+1);
	return (H<<8)|L;   //合成数据
}



/*************************************
 * 函数名：Read_Filter_MPU6050
 * 描述  ：读MPU6050的加速度与角速度的值，
					 如果是第一次调用该函数（开机），
					 会计算零偏，其他时候不计算；
					 滑动滤波；
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 ************************************/
uint8_t Read_Filter_MPU6050(void)
{
	static uint8_t	GYRO_OFFSET_OK = 0;
    //static uint8_t	ACC_OFFSET_OK  = 0;
	static uint8_t filter_cnt= 0;
  	static int16_t	ACC_X_BUF[MPU6050_FILTER_NUM],ACC_Y_BUF[MPU6050_FILTER_NUM],ACC_Z_BUF[MPU6050_FILTER_NUM];		
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;
	
	ACC_OFFSET.X=-5;
	ACC_OFFSET.Y=-160;
	ACC_OFFSET.Z=0;
	#if 0
	GYRO_OFFSET.X=-150;
	GYRO_OFFSET.Y=-33;
	GYRO_OFFSET.Z=15;
	#endif

	MPU6050_ACC_LAST.X=GetData_MPU6050(ACCEL_XOUT_H)- ACC_OFFSET.X;
	MPU6050_ACC_LAST.Y=GetData_MPU6050(ACCEL_YOUT_H)- ACC_OFFSET.Y;
	MPU6050_ACC_LAST.Z=GetData_MPU6050(ACCEL_ZOUT_H)- ACC_OFFSET.Z;
	
	MPU6050_GYRO_LAST.X=GetData_MPU6050(GYRO_XOUT_H)- GYRO_OFFSET.X;
	MPU6050_GYRO_LAST.Y=GetData_MPU6050(GYRO_YOUT_H)- GYRO_OFFSET.Y;
	MPU6050_GYRO_LAST.Z=GetData_MPU6050(GYRO_ZOUT_H)- GYRO_OFFSET.Z;
	if(!GYRO_OFFSET_OK)
	{
		static int32_t	tempgx=0,tempgy=0,tempgz=0;
		static uint8_t cnt_g=0;
		if(cnt_g==0)
		{
			GYRO_OFFSET.X=0;
			GYRO_OFFSET.Y=0;
			GYRO_OFFSET.Z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;
			return 0;
		}
		tempgx+= MPU6050_GYRO_LAST.X;
		tempgy+= MPU6050_GYRO_LAST.Y;
		tempgz+= MPU6050_GYRO_LAST.Z;
		if(cnt_g==200)
		{
			GYRO_OFFSET.X=tempgx/cnt_g;
			GYRO_OFFSET.Y=tempgy/cnt_g;
			GYRO_OFFSET.Z=tempgz/cnt_g;
			cnt_g = 0;
			GYRO_OFFSET_OK = 1;
			return 1;
		}
		cnt_g++;
		}
	/*
	if(!ACC_OFFSET_OK)
	{
		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint8_t cnt_a=0;
		if(cnt_a==0)
		{
			ACC_OFFSET.X = 0;
			ACC_OFFSET.Y = 0;
			ACC_OFFSET.Z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return 0;
		}
		tempax+= MPU6050_ACC_LAST.X;
		tempay+= MPU6050_ACC_LAST.Y;
		tempaz+= MPU6050_ACC_LAST.Z;
		if(cnt_a==200)
		{
			ACC_OFFSET.X=tempax/cnt_a;
			ACC_OFFSET.Y=tempay/cnt_a;
			ACC_OFFSET.Z=tempaz/cnt_a;
			cnt_a = 0;
			ACC_OFFSET_OK = 1;
			return 1;
		}
		cnt_a++;		
	}
	*/    
  	ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST.X;
	ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST.Y;
	ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST.Z;
	for(i=0;i<MPU6050_FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_AVG.X = temp1 / MPU6050_FILTER_NUM;
	ACC_AVG.Y = temp2 / MPU6050_FILTER_NUM;
	ACC_AVG.Z = temp3 / MPU6050_FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==MPU6050_FILTER_NUM)	filter_cnt=0;
	return 1;
}

/*************************************
 * 函数名：Cal_Offset_MPU6050
 * 描述  ：计算MPU6050的加速度和角速度的零偏值
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 ************************************/
 /*
void Cal_Offset_MPU6050(void)
{
	ACC_OFFSET.X=GetData_MPU6050(ACCEL_XOUT_H);
	ACC_OFFSET.Y=GetData_MPU6050(ACCEL_YOUT_H);
	ACC_OFFSET.Z=GetData_MPU6050(ACCEL_ZOUT_H);
	
	GYRO_OFFSET.X=GetData_MPU6050(GYRO_XOUT_H);
	GYRO_OFFSET.Y=GetData_MPU6050(GYRO_YOUT_H);
	GYRO_OFFSET.Z=GetData_MPU6050(GYRO_ZOUT_H);
}
*/
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


/**************************************
 * 函数名：void HMC5883L_Init(void)
 * 描述  ：初始化HMC5883L
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void HMC5883L_Init(void)
{
	
	I2C_ByteWrite(MPU6050_SlaveAddress,MPU6050_INT_PIN_CFG,MPU6050_I2C_BYPASS_EN);
	/*使能bypass*/
	OSTimeDly(10);	
	I2C_ByteWrite(HMC5883L_WRITE_ADDRESS,HMC5883L_CONFIG_A,0x78);
	OSTimeDly(10);
	I2C_ByteWrite(HMC5883L_WRITE_ADDRESS,HMC5883L_MODE,0x00);
	OSTimeDly(10);	
}

/*************************************
 * 函数名：GetData_HMC5883L
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ************************************/
    
uint16_t GetData_HMC5883L(uint16_t REG_Address)
{
	uint8_t H,L;
	u16 temp;
	H=I2C_ByteRead(HMC5883L_READ_ADDRESS,REG_Address);
	L=I2C_ByteRead(HMC5883L_READ_ADDRESS,REG_Address+1);
	temp=(H<<8)|L;   //合成数据
	return temp>0x7fff?temp-=0xffff:temp;//求原码
}

S_INT16_XYZ HMC5883L_MAGN_LAST;

/*************************************
 * 函数名：Read_HMC5883L
 * 描述  ：获得16位数据，求得磁场的X,Y,Z三轴的分量
 * 输入  ：无
 * 输出  ：Z轴角度
 * 调用  ：外部调用
 ************************************/
float Read_HMC5883L(void)//读HMC5883L的数据，输出为16位经过整合的
{
	
	HMC5883L_MAGN_LAST.X=GetData_HMC5883L(HMC5883L_X_MSB);
	HMC5883L_MAGN_LAST.Y=GetData_HMC5883L(HMC5883L_Y_MSB);
	HMC5883L_MAGN_LAST.Z=GetData_HMC5883L(HMC5883L_Z_MSB);
	return (atan2(HMC5883L_MAGN_LAST.Y,HMC5883L_MAGN_LAST.X)*(57.2957796)-180);
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
float H=0;
float P1=0,P2=0;

uint32_t  Cal_C[7];	        //用于存放PROM中的8组数据
uint32_t D1_Pres,D2_Temp;	// 存放压力和温度

uint64_t dT,TEMP;
uint64_t OFF_,SENS;
uint32_t Pressure;				//大气压
uint32_t TEMP2,Aux,OFF2,SENS2;	//温度校验值


/*******************************************************************************
  * @函数名称	MS5611_Init
  * @函数说明   初始化5611
  * @输入参数  	无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
//void MS5611_Init(void)
//{
//	uint8_t i;
//	I2C_ByteWrite_Ms5611(MS561101BA_SlaveAddress,MS561101BA_RST);
//	delay_ms(20);
//	for(i=1;i<=6;i++)
//	{
//		I2C_ByteWrite_Ms5611(MS561101BA_SlaveAddress,MS561101BA_PROM_RD+i*2);
//		delay_ms(10);
//		Cal_C[i]=I2C_ByteRead_Ms5611_16BIT(MS561101BA_SlaveAddress+1);
//		delay_ms(10);
//	}
//	I2C_ByteWrite_Ms5611(MS561101BA_SlaveAddress,MS5611_OSR4096+0x10);
//	delay_ms(10);
//}

///*******************************************************************************
//  * @函数名称		 MS5611_Read24bits
//  * @函数说明   
//  * @输入参数  	
//  * @输出参数   24位温度值或者压强值
//  * @返回参数   无
//*******************************************************************************/
//uint32_t MS5611_Read24bits(uint8_t command)
//{
//		
//	uint32_t Read_Temp=0;
//	
//	I2C_ByteWrite_Ms5611(MS561101BA_SlaveAddress,0x00);
//	
//	Read_Temp=I2C_ByteRead_Ms5611_24BIT(MS561101BA_SlaveAddress+1);
//	
//	I2C_ByteWrite_Ms5611(MS561101BA_SlaveAddress,command);//触发ADC开始转换数据；
//	
//	return Read_Temp;
//}


///*******************************************************************************
//  * @函数名称	MS5611_GetTemperature
//  * @函数说明   获取温度值
//  * @输入参数  	转换频率
//  * @输出参数   无
//  * @返回参数   温度
//*******************************************************************************/
//float MS5611_GetTemperature(uint8_t OSR_Temp)
//{  

//	float T_float;
//	D2_Temp= MS5611_Read24bits(OSR_Temp);
//	delay_us(10);
//	dT=D2_Temp - (Cal_C[5]<<8);
//	TEMP=2000+((dT*Cal_C[6])>>23);
//	T_float=TEMP/100.0;
//	return T_float;
//}



///*******************************************************************************
//  * @函数名称	MS5611_GetPressure
//  * @函数说明   获得压强值
//  * @输入参数  	 转换频率
//  * @输出参数   无
//  * @返回参数   压强
//*******************************************************************************/
//float MS5611_GetPressure(uint8_t OSR_Pres)
//{
//	float P_float;
//	D1_Pres= MS5611_Read24bits(OSR_Pres);
//	delay_us(10);
//	OFF_=(Cal_C[2]<<16)+((Cal_C[4]*dT)>>7);
//	SENS=(Cal_C[1]<<15)+((Cal_C[3]*dT)>>8);
//	
//	if(TEMP<2000)//温度补偿压强
//	{
//		Aux = TEMP*TEMP;
//		OFF2 = 2.5*Aux;
//		SENS2 = 1.25*Aux;
//		
//		TEMP = TEMP - TEMP2;
//		OFF_ = OFF_ - OFF2;
//		SENS = SENS - SENS2;	
//	}

//	Pressure=((((D1_Pres*SENS)>>21)-OFF_)>>15);	
//	P_float=Pressure;
//	return P_float;
//}


///*******************************************************************************
//  * @函数名称	Get_High
//  * @函数说明   得到高度
//  * @输入参数  	转换频率
//  * @输出参数   高度
//  * @返回参数   高度
//*******************************************************************************/
//float Get_High(uint8_t OSR)//OSR为压强转换速率
//{
//	static u8 filter_cnt_t=0,filter_cnt_p=0,sample=0;
//	static float t=0;
//	static float P_BUF[MSS5611_FILTER_NUM];
//  	static float T_BUF[MSS5611_FILTER_NUM];
//	u8 i;
//	float P_TEMP=0;
//	float T_TEMP=0;

//	sample++;
//	
//	if(sample==1)
//	{

//		t=MS5611_GetTemperature(OSR);//触发压强读数，进行ADC转换
//		T_BUF[filter_cnt_t]=t;		
//		for(i=0;i<MSS5611_FILTER_NUM;i++)
//		{
//			T_TEMP+=T_BUF[i];
//		}
//		t=T_TEMP/MSS5611_FILTER_NUM; /*对温度进行平滑滤波,该温度值后面计算高度要用*/
//		filter_cnt_t++;
//		if(filter_cnt_t==MSS5611_FILTER_NUM)	filter_cnt_t=0;
//	}
//	
//	if(sample==2)
//	{
//		sample=0;
//		P2=MS5611_GetPressure(OSR+0x10);//触发温度读数，进行ADC转换	
//		P_BUF[filter_cnt_p]=P2;		
//		for(i=0;i<MSS5611_FILTER_NUM;i++)
//		{
//			P_TEMP+=P_BUF[i];
//		}
//		P2=P_TEMP/MSS5611_FILTER_NUM; /*对压强进行平滑滤波*/
//		filter_cnt_p++;
//		if(filter_cnt_p==MSS5611_FILTER_NUM)	filter_cnt_p=0;
//	}          
//	
//	return 18400*(1+t/273.0)*log(P1/P2);
//	
//}
