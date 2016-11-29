/***************************************************************************************************
  * �� �� ��: mpu6050.c
  * ��    �ߣ�����
  *	��    �ڣ�2014/7/10
  * ��    ����V1.0
  * �� �� �壺TM4C123G6PZ--ʵ�鿪����
  * �޸�ʱ�䣺�� 
  * �� �� ��: keil.4.54
  *-------------------------------------------------------------------------------------------------
  * ��    ��:
  * ��    ��:
  *	�ʼ�����:
  * ע    �⣺
  *************************************************************************************************/  
/*--------����ͷ�ļ�------------------------------------------------------------------------------*/
#include "mpu6050.h"

/**************************************************************************************************/
/*--------�����Ķ���-------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------�ṹ��Ķ���-------------------------------------------------------------------------------*/
/* MPU6050 Register Address ------------------------------------------------------------*/
#define	SMPLRT_DIV		0x19	//25����Ƶ�ʷ�Ƶ��//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG				0x1A	//26����//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//27����������//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//28���ٶȼ�����//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
// 	 
#define	ACCEL_XOUT_H	0x3B	//59
#define	ACCEL_XOUT_L	0x3C	//60
#define	ACCEL_YOUT_H	0x3D	//61
#define	ACCEL_YOUT_L	0x3E  //62
#define	ACCEL_ZOUT_H	0x3F	//63
#define	ACCEL_ZOUT_L	0x40	//64//���ٶȼƲ���ֵaccelerometer measurements
	 
#define	TEMP_OUT_H		0x41  //65
#define	TEMP_OUT_L		0x42	//66//temperature measurement

#define	GYRO_XOUT_H		0x43	//67
#define	GYRO_XOUT_L		0x44	//68
#define	GYRO_YOUT_H		0x45	//69
#define	GYRO_YOUT_L		0x46	//70
#define	GYRO_ZOUT_H		0x47	//71
#define	GYRO_ZOUT_L		0x48	//72//gyroscope measurements

#define	PWR_MGMT_1		0x6B	//107//��Դ��������ֵ��0x00(��������)power management 1
// #define	WHO_AM_I			0x75	//117//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

// #define	SlaveAddress_simulation	0xD0	//IICд��ʱ�ĵ�ַ�ֽ�����
// /**************************************************************************************************/
#define	MPU6050_Addr   0xD0	  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

/*-------���ں���----------------------------------------------------------------------------------*/
uint8_t Single_Read(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation);
uint8_t Single_Write(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation,uint8_t REG_data_simulation);

/**************************************************************************************************/
/*--------����ģ��----------------------------------------------------------------------------------*/

#define  Init_Timers  6                     // ���ݳ�ʼ������

MPU6050_Typedef _6050_Init_Data;                             // 6050��ʼ������

// �������˲�
#define  GyroFilter_Len  3                   // �������˲�����
const float GyroFilter_Factor[GyroFilter_Len]= {0.65,0.225,0.125};           // ��������������� Ԫ��0
 int16_t _6050_GyroFilter_Buff[3][GyroFilter_Len];

// ���ٶ��˲�
#define  AccFilter_Len  10                   // ���ٶȼ��˲�����
const float AccFilter_Factor[AccFilter_Len]= {0.4,0.2,0.12,0.08,0.06,0.04,0.035,0.03,0.02,0.015};             // ����ļ��ٶȼ����� Ԫ��0
 int16_t _6050_AccFilter_Buff[3][AccFilter_Len];

/**************************************************************************************************/
/*--------�ṹ��Ķ���-------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*-------���ں���----------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------����ģ��----------------------------------------------------------------------------------*/
/***************************************************************************************************
	������: ��ʼ��MPU6050
	��  ��: 
	��  ��:	
	��  ��:	
****************************************************************************************************/	
void Congfigration_mpu6050(void)
{
	Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	//�������״̬
	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x07);	//ȡ����2��Ƶ��1k/2=500hz
	Single_Write(MPU6050_Addr,CONFIG, 0x06);	  	//��ͨ�˲�94hz
	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);	//2000dps
	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01); 	//+-8g,5Hz��ͨ�˲�	
}
/***********************************************************************************************************
 * ��������GetData
 * ����  �����16λ����
 * ����  ��REG_Address �Ĵ�����ַ
 * ���  �����ؼĴ�������
 * ����  ���ⲿ����
 ***********************************************************************************************************/
uint16_t GetData(unsigned char REG_Address)
{
	char H,L;
	H=Single_Read(MPU6050_Addr,REG_Address);
	L=Single_Read(MPU6050_Addr,REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}

//--------------------------- Ӧ�ò����� --------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//  & �������� MPU6050ԭʼ���ݻ�ȡ
//  & ��  ���� 6050���ݽṹ��
//  & ��  ��;  ��
//  ----------------------------------------------------------------------------------------------
//  & ˵  ���� ��
//-------------------------------------------------------------------------------------------------
void Get_MPU6050_firstData(volatile MPU6050_Typedef* _6050data)
{
	_6050data->Gyro_Data[0] = (int16_t)GetData(GYRO_XOUT_H) - _6050_Init_Data.Gyro_Data[0];  // ����������-> ���ٶ�
	_6050data->Gyro_Data[1] = (int16_t)GetData(GYRO_YOUT_H) - _6050_Init_Data.Gyro_Data[1];
	_6050data->Gyro_Data[2] = (int16_t)GetData(GYRO_ZOUT_H) - _6050_Init_Data.Gyro_Data[2];
	_6050data->Acc_Data[0] = -(int16_t)GetData(ACCEL_XOUT_H) - _6050_Init_Data.Acc_Data[0];  // ���ٶȴ��������� -> ���ٶ�
	_6050data->Acc_Data[1] = -(int16_t)GetData(ACCEL_YOUT_H) - _6050_Init_Data.Acc_Data[1];
	_6050data->Acc_Data[2] = -(int16_t)GetData(ACCEL_ZOUT_H);        // Z����ٶȣ���ȥ���
}

//-------------------------------------------------------------------------------------------------
//  & �������� MPU6050�����˲���������ȡ
//  & ��  ���� 6050���ݽṹ��
//  & ��  ��;  ��
//  ----------------------------------------------------------------------------------------------
//  & ˵  ���� ��
//-------------------------------------------------------------------------------------------------
void Get_MPU6050_FilterAfter_Data(volatile MPU6050_Typedef* _6050data)
{
	int8_t mpu_i,mpu_j;
	float temp;
	Get_MPU6050_firstData(_6050data);
	
	// ������
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
	// ���ٶ�
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
//  & �������� MPU6050 ���ݳ�ʼ��
//  & ��  ���� ��
//  & ��  ��;  ��
//  ----------------------------------------------------------------------------------------------
//  & ˵  ���� ��
//-------------------------------------------------------------------------------------------------
void MPU6050_Init(void)
{
	uint8_t _6050_i,_6050_j;
	float _6050_TempDataG[3],_6050_TempDataA[2];
	MPU6050_Typedef _6050_TTemp;
	
	//--------- �������ó�ʼ�� ----------------------------------------------
	Simulation_II2C_GPIO_Config();
	delay_ms(1);
	Congfigration_mpu6050();
	delay_ms(20);
	// �������㹤��
	for(_6050_i=0; _6050_i<3;_6050_i++)
	{
		_6050_Init_Data.Gyro_Data[_6050_i] =0;
		_6050_Init_Data.Acc_Data[_6050_i] =0;
		_6050_TempDataG[_6050_i] =0;
	}
	_6050_TempDataA[0] =0;
	_6050_TempDataA[1] =0;
	
	// ��ƽ��ֵ
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
	
	// �˲�������������װ��
	// �������˲�
	for(_6050_i =0; _6050_i< 3; _6050_i++)
	{
		for(_6050_j =0; _6050_j< GyroFilter_Len; _6050_j++)
			_6050_GyroFilter_Buff[_6050_i][_6050_j] =0;
	}
	// ���ٶ��˲�
	for(_6050_i =0; _6050_i< 2; _6050_i++)
	{
		for(_6050_j =0; _6050_j< AccFilter_Len; _6050_j++)
			_6050_AccFilter_Buff[_6050_i][_6050_j] =0;
	}
	for(_6050_j =0; _6050_j< AccFilter_Len; _6050_j++)
			_6050_AccFilter_Buff[2][_6050_j] =_6050_TTemp.Acc_Data[2];
}

//===================== �ײ�I2C Э�� ===============================================================
/*ģ��IIC�˿�������붨��*/
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
	if(!SDA_read)return FALSE;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	HWREG(0x40004400) |= 0x00000040;
	SDA_L;
	I2C_delay();
// 	if(SDA_read) return FALSE;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
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
bool I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
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
void I2C_SendByte(uint8_t SendByte) //���ݴӸ�λ����λ//
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
unsigned char I2C_RadeByte(void)  //���ݴӸ�λ����λ//
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
//���ֽ�д��*******************************************

uint8_t Single_Write(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation,uint8_t REG_data_simulation)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress_simulation);   //�����豸��ַ+д�ź�//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//���ø���ʼ��ַ+������ַ 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address_simulation );   //���õ���ʼ��ַ      
    I2C_WaitAck();	
    I2C_SendByte(REG_data_simulation);
    I2C_WaitAck();   
    I2C_Stop(); 
//    delay_ms(5);
    return TRUE;
}

//���ֽڶ�ȡ*****************************************
uint8_t Single_Read(uint8_t SlaveAddress_simulation,uint8_t REG_Address_simulation)
{   uint8_t REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress_simulation); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte((uint8_t) REG_Address_simulation);   //���õ���ʼ��ַ      
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

/*****************************@��Ȩ luomin*********************************************************/
