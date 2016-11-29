/***************************************************************************************************
  * 文 件 名: mpu6050.c
  * 作    者：罗民
  *	日    期：2014/7/10
  * 版    本：V1.0
  * 开 发 板：TM4C123G6PZ--实验开发板
  * 修改时间：无 
  * 编 译 器: keil.4.54
  *-------------------------------------------------------------------------------------------------
  * 简    介:
  * 参    考:
  *	笔记名称:
  * 注    意：
  *************************************************************************************************/  
/*--------包含头文件------------------------------------------------------------------------------*/
#include "mpu6050.h"

/**************************************************************************************************/
/*--------变量的定义-------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------结构体的定义-------------------------------------------------------------------------------*/
/* MPU6050 Register Address ------------------------------------------------------------*/
#define	SMPLRT_DIV		0x19	//25采样频率分频器//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG				0x1A	//26配置//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//27陀螺仪配置//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//28加速度计配置//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
// 	 
#define	ACCEL_XOUT_H	0x3B	//59
#define	ACCEL_XOUT_L	0x3C	//60
#define	ACCEL_YOUT_H	0x3D	//61
#define	ACCEL_YOUT_L	0x3E  //62
#define	ACCEL_ZOUT_H	0x3F	//63
#define	ACCEL_ZOUT_L	0x40	//64//加速度计测量值accelerometer measurements
	 
#define	TEMP_OUT_H		0x41  //65
#define	TEMP_OUT_L		0x42	//66//temperature measurement

#define	GYRO_XOUT_H		0x43	//67
#define	GYRO_XOUT_L		0x44	//68
#define	GYRO_YOUT_H		0x45	//69
#define	GYRO_YOUT_L		0x46	//70
#define	GYRO_ZOUT_H		0x47	//71
#define	GYRO_ZOUT_L		0x48	//72//gyroscope measurements

#define	PWR_MGMT_1		0x6B	//107//电源管理，典型值：0x00(正常启用)power management 1
// #define	WHO_AM_I			0x75	//117//IIC地址寄存器(默认数值0x68，只读)

// #define	SlaveAddress_simulation	0xD0	//IIC写入时的地址字节数据
// /**************************************************************************************************/
#define	MPU6050_Addr   0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

/*-------进口函数----------------------------------------------------------------------------------*/
uint8_t Single_Read(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation);
uint8_t Single_Write(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation,uint8_t REG_data_simulation);

/**************************************************************************************************/
/*--------函数模型----------------------------------------------------------------------------------*/

#define  Init_Timers  6                     // 数据初始化次数

MPU6050_Typedef _6050_Init_Data;                             // 6050初始化数据

// 陀螺仪滤波
#define  GyroFilter_Len  3                   // 陀螺仪滤波次数
const float GyroFilter_Factor[GyroFilter_Len]= {0.65,0.225,0.125};           // 最近的陀螺仪因数 元素0
 int16_t _6050_GyroFilter_Buff[3][GyroFilter_Len];

// 加速度滤波
#define  AccFilter_Len  10                   // 加速度计滤波次数
const float AccFilter_Factor[AccFilter_Len]= {0.4,0.2,0.12,0.08,0.06,0.04,0.035,0.03,0.02,0.015};             // 最近的加速度计因数 元素0
 int16_t _6050_AccFilter_Buff[3][AccFilter_Len];

/**************************************************************************************************/
/*--------结构体的定义-------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*-------进口函数----------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------函数模型----------------------------------------------------------------------------------*/
/***************************************************************************************************
	函数名: 初始化MPU6050
	输  入: 
	输  出:	
	功  能:	
****************************************************************************************************/	
void Congfigration_mpu6050(void)
{
	Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态
	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x07);	//取样率2分频，1k/2=500hz
	Single_Write(MPU6050_Addr,CONFIG, 0x06);	  	//低通滤波94hz
	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);	//2000dps
	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01); 	//+-8g,5Hz高通滤波	
}
/***********************************************************************************************************
 * 函数名：GetData
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ***********************************************************************************************************/
uint16_t GetData(unsigned char REG_Address)
{
	char H,L;
	H=Single_Read(MPU6050_Addr,REG_Address);
	L=Single_Read(MPU6050_Addr,REG_Address+1);
	return (H<<8)+L;   //合成数据
}

//--------------------------- 应用层驱动 --------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//  & 函数名： MPU6050原始数据获取
//  & 参  数： 6050数据结构体
//  & 返  回;  无
//  ----------------------------------------------------------------------------------------------
//  & 说  明： 无
//-------------------------------------------------------------------------------------------------
void Get_MPU6050_firstData(volatile MPU6050_Typedef* _6050data)
{
	_6050data->Gyro_Data[0] = (int16_t)GetData(GYRO_XOUT_H) - _6050_Init_Data.Gyro_Data[0];  // 陀螺仪数据-> 角速度
	_6050data->Gyro_Data[1] = (int16_t)GetData(GYRO_YOUT_H) - _6050_Init_Data.Gyro_Data[1];
	_6050data->Gyro_Data[2] = (int16_t)GetData(GYRO_ZOUT_H) - _6050_Init_Data.Gyro_Data[2];
	_6050data->Acc_Data[0] = -(int16_t)GetData(ACCEL_XOUT_H) - _6050_Init_Data.Acc_Data[0];  // 加速度传感器数据 -> 加速度
	_6050data->Acc_Data[1] = -(int16_t)GetData(ACCEL_YOUT_H) - _6050_Init_Data.Acc_Data[1];
	_6050data->Acc_Data[2] = -(int16_t)GetData(ACCEL_ZOUT_H);        // Z轴加速度，不去零点
}

//-------------------------------------------------------------------------------------------------
//  & 函数名： MPU6050简易滤波后数据提取
//  & 参  数： 6050数据结构体
//  & 返  回;  无
//  ----------------------------------------------------------------------------------------------
//  & 说  明： 无
//-------------------------------------------------------------------------------------------------
void Get_MPU6050_FilterAfter_Data(volatile MPU6050_Typedef* _6050data)
{
	int8_t mpu_i,mpu_j;
	float temp;
	Get_MPU6050_firstData(_6050data);
	
	// 陀螺仪
	for(mpu_i=0;mpu_i<3;mpu_i++)
	{
		temp=0;
		for(mpu_j=GyroFilter_Len-1;mpu_j>0;mpu_j--)
		{
			_6050_GyroFilter_Buff[mpu_i][mpu_j] = _6050_GyroFilter_Buff[mpu_i][mpu_j-1];
			temp += _6050_GyroFilter_Buff[mpu_i][mpu_j]*GyroFilter_Factor[mpu_j];
		}
		_6050_GyroFilter_Buff[mpu_i][0] = _6050data->Gyro_Data[mpu_i];
		temp += _6050data->Gyro_Data[mpu_i]*GyroFilter_Factor[0];
		_6050data->Gyro_Data[mpu_i] = (int16_t)temp;
	}
	// 加速度
	for(mpu_i=0;mpu_i<3;mpu_i++)
	{
		temp=0;
		for(mpu_j=AccFilter_Len-1;mpu_j>0;mpu_j--)
		{
			_6050_AccFilter_Buff[mpu_i][mpu_j] = _6050_AccFilter_Buff[mpu_i][mpu_j-1];
			temp += _6050_AccFilter_Buff[mpu_i][mpu_j]*AccFilter_Factor[mpu_j];
		}
		_6050_AccFilter_Buff[mpu_i][0] = _6050data->Acc_Data[mpu_i];
		temp += _6050data->Acc_Data[mpu_i]*AccFilter_Factor[0];
		_6050data->Acc_Data[mpu_i] = (int16_t)temp;
	}
}

//-------------------------------------------------------------------------------------------------
//  & 函数名： MPU6050 数据初始化
//  & 参  数： 无
//  & 返  回;  无
//  ----------------------------------------------------------------------------------------------
//  & 说  明： 无
//-------------------------------------------------------------------------------------------------
void MPU6050_Init(void)
{
	uint8_t _6050_i,_6050_j;
	float _6050_TempDataG[3],_6050_TempDataA[2];
	MPU6050_Typedef _6050_TTemp;
	
	//--------- 基本配置初始化 ----------------------------------------------
	Simulation_II2C_GPIO_Config();
	delay_ms(1);
	Congfigration_mpu6050();
	delay_ms(20);
	// 数据清零工作
	for(_6050_i=0; _6050_i<3;_6050_i++)
	{
		_6050_Init_Data.Gyro_Data[_6050_i] =0;
		_6050_Init_Data.Acc_Data[_6050_i] =0;
		_6050_TempDataG[_6050_i] =0;
	}
	_6050_TempDataA[0] =0;
	_6050_TempDataA[1] =0;
	
	// 求平均值
	for(_6050_i =0; _6050_i< Init_Timers; _6050_i++)
	{
		delay_ms(2);
		Get_MPU6050_firstData(&_6050_TTemp);
		_6050_TempDataG[0] += _6050_TTemp.Gyro_Data[0];
		_6050_TempDataG[1] += _6050_TTemp.Gyro_Data[1];
		_6050_TempDataG[2] += _6050_TTemp.Gyro_Data[2];
		
		_6050_TempDataA[0] += _6050_TTemp.Acc_Data[0];
		_6050_TempDataA[1] += _6050_TTemp.Acc_Data[1];
	}
	
	_6050_Init_Data.Gyro_Data[0] = (int16_t)(_6050_TempDataG[0]/Init_Timers);
	_6050_Init_Data.Gyro_Data[1] = (int16_t)(_6050_TempDataG[1]/Init_Timers);
	_6050_Init_Data.Gyro_Data[2] = (int16_t)(_6050_TempDataG[2]/Init_Timers);
	
	_6050_Init_Data.Acc_Data[0] = 0;//0x0700;//0x0800;//(int16_t)(_6050_TempDataA[0]/Init_Timers);
	_6050_Init_Data.Acc_Data[1] = 0;//0x0300;//0x0330;//(int16_t)(_6050_TempDataA[1]/Init_Timers);
	
	// 滤波器缓冲区数据装载
	// 陀螺仪滤波
	for(_6050_i =0; _6050_i< 3; _6050_i++)
	{
		for(_6050_j =0; _6050_j< GyroFilter_Len; _6050_j++)
			_6050_GyroFilter_Buff[_6050_i][_6050_j] =0;
	}
	// 加速度滤波
	for(_6050_i =0; _6050_i< 2; _6050_i++)
	{
		for(_6050_j =0; _6050_j< AccFilter_Len; _6050_j++)
			_6050_AccFilter_Buff[_6050_i][_6050_j] =0;
	}
	for(_6050_j =0; _6050_j< AccFilter_Len; _6050_j++)
			_6050_AccFilter_Buff[2][_6050_j] =_6050_TTemp.Acc_Data[2];
}

//===================== 底层I2C 协议 ===============================================================
/*模拟IIC端口输出输入定义*/
#define SDA_read      GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6)

/***************************************************************************************************
* Function Name		 : I2C_GPIO_CONFIG
* Description	     : Configration simulation IIC gpio
* Input            : None
* Output           : None
****************************************************************************************************/	
void Simulation_II2C_GPIO_Config(void)
{
  OUT_NormalGPIO_Conf(GPIOA,GPIO_PIN_6);
	OUT_NormalGPIO_Conf(GPIOA,GPIO_PIN_7);
}
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
	__nop();
}
/***************************************************************************************************
* Function Name		 : I2C_start
* Description	     : Master Start Simulation IIC Communication
* Input            : None
* Output           : None
****************************************************************************************************/	
bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	HWREG(0x40004400) &= 0xffffffbf; 
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	HWREG(0x40004400) |= 0x00000040;
	SDA_L;
	I2C_delay();
// 	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
bool I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	HWREG(0x40004400) &= 0xffffffbf; 
	if(SDA_read)
	{
      SCL_L;
	    I2C_delay();
      return FALSE;
	}
	HWREG(0x40004400) |= 0x00000040;
	SCL_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(uint8_t SendByte) //数据从高位到低位//
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H  
      else 
        SDA_L  
        SendByte<<=1;
        I2C_delay();
		    SCL_H;
        I2C_delay();
    }
    SCL_L;
} 
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{  
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;	
		while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	    SCL_H;
      I2C_delay();	
			HWREG(0x40004400) &= 0xffffffbf; 
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
		HWREG(0x40004400) |= 0x00000040;
    SCL_L;
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************

uint8_t Single_Write(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation,uint8_t REG_data_simulation)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress_simulation);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address_simulation );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data_simulation);
    I2C_WaitAck();   
    I2C_Stop(); 
//    delay_ms(5);
    return TRUE;
}

//单字节读取*****************************************
uint8_t Single_Read(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation)
{   uint8_t REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress_simulation); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte((uint8_t) REG_Address_simulation);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress_simulation+1);
    I2C_WaitAck();

	REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;
}	

/*****************************@版权 luomin*********************************************************/
