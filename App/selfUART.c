
#include "selfuart.h"

void UART0_Conf(uint32_t Band)			//USART0����    PA0--Rx   PA1--Tx
{
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);		//USART0 ʱ��ʹ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	//PA ʱ��ʹ��

	  GPIOPinConfigure(GPIO_PA0_U0RX);			//��PA0��Ϊusart���ù���
    GPIOPinConfigure(GPIO_PA1_U0TX);		//��PA1��Ϊusart���ù���
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);		//����usart
	  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), Band,		//16M ������
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
		UARTIntEnable(UART0_BASE,UART_INT_RX);			//8λ����λ   1λֹͣλ   ��У��
		UARTFIFODisable(UART0_BASE);		//ʹ��usart0����    ʧ��fifoģʽ
}


uint8_t data_to_send[30];
extern volatile MPU6050_Typedef _6050_Data;	//mpu6050ԭʼ����
extern volatile Angle_Typedef _Angle_Data;		//ŷ��������
extern Control_Signal_Tpedef _PPM_Data;	//ppm����
extern uint16_t out[4];			//���
//extern PID Roll,Pitch,Yaw;
void Data_Send_Status(void)
{
	uint8_t _cnt=0;
	uint16_t _temp;
	uint8_t sum = 0;
	uint8_t i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(_Angle_Data.Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(-_Angle_Data.Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(_Angle_Data.Yaw*100);			//��ʾ������
	//_temp = (int)(Mag_Heading*100);
	data_to_send[_cnt++]=108;
	data_to_send[_cnt++]=108;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
		
/*	if(Rc_C.ARMED==0)		data_to_send[_cnt++]=0xA0;	
	else if(Rc_C.ARMED==1)		data_to_send[_cnt++]=0xA1;*/data_to_send[_cnt++]=0xA1;
	
	data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Uart1_Send_Buf(data_to_send,_cnt);
}

void Data_Send_RCData(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(_PPM_Data.EXP_Throt);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Throt);
	data_to_send[_cnt++]=BYTE1(_PPM_Data.EXP_Yaw);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Yaw);
	data_to_send[_cnt++]=BYTE1(_PPM_Data.EXP_Roll);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Roll);
	data_to_send[_cnt++]=BYTE1(_PPM_Data.EXP_Pitch);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Pitch);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE0(_PPM_Data.NSWF);
	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=BYTE0(_PPM_Data.Mode);
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Uart1_Send_Buf(data_to_send,_cnt);
}

void Data_Send_Senser(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	int _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
// 	_temp=(int)(_6050_Data.Acc_Data[0]/1.671836);
	_temp=(int)(Roll_PID.PIDInter_Sum);
	data_to_send[_cnt++]=BYTE1(_temp);  
	data_to_send[_cnt++]=BYTE0(_temp); 
// 	_temp=(int)(_6050_Data.Acc_Data[1]/1.671836);
	_temp=(int)(Pitch_PID.PIDInter_Sum);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp=(int)(_6050_Data.Acc_Data[2]/1.671836);
	_temp=(int)(Yaw_PID.PIDInter_Sum);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
																						//GYRO_X=_PPM_Data.EXP_Roll
	data_to_send[_cnt++]=BYTE1(_PPM_Data.EXP_Roll);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Roll);
																						//GYRO_Y=_PPM_Data.EXP_Pitch
	data_to_send[_cnt++]=BYTE1(_PPM_Data.EXP_Pitch);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Pitch);
																								//GYRO_Z=_PPM_Data.EXP_Yaw
	data_to_send[_cnt++]=BYTE1(_PPM_Data.EXP_Yaw);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Yaw);
																								//MAG_X=_PPM_Data.EXP_Throt
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Throt);
	data_to_send[_cnt++]=BYTE0(_PPM_Data.EXP_Throt);
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;	//MAG_Y=Roll.OUT;
											
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	Uart1_Send_Buf(data_to_send,_cnt);
}


void Data_Send_MotoPWM(void)
{
	u8 _cnt=0;
	u8 i=0;
	u8 sum = 0;
//	for(i=0;i<4;i++)
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart1_Send_Buf(data_to_send,_cnt);
	
}

void Data_Send_PID1(void)
{
	u8 _cnt=0;
	u16 _temp;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	_temp = Roll_PID.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Roll_PID.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Roll_PID.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch_PID.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch_PID.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch_PID.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw_PID.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw_PID.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw_PID.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart1_Send_Buf(data_to_send,_cnt);
}

void Data_Send_PID2(void)
{
	u8 _cnt=0;
	u16 _temp;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
//	_temp = Height_PID.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = Height_PID.Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = Height_PID.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart1_Send_Buf(data_to_send,_cnt);
}

void Data_Send_Check(u16 check)
{
	u8 sum = 0;
	u8 i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=BYTE1(check);
	data_to_send[6]=BYTE0(check);
	
	for( i=0;i<7;i++)
		sum += data_to_send[i];
	
	data_to_send[7]=sum;

	Uart1_Send_Buf(data_to_send,8);
}

void Uart1_Send_Buf(uint8_t *buf,uint8_t len)		//����buf,����len,�����ֽں�sum
{
	while(len)
	{
		UARTCharPut(UART0_BASE,*buf);
		buf++;
		len--;
	}
}


void Show_Data(void)
{
	static uint8_t _cnt=1;
	static	uint16_t _cnt_pid=0;	
	switch(_cnt)
	{	
	case 1:
		_cnt=0;
		Data_Send_Status();				//��λ����ʾ��̬
		break;
	case 2:
		Data_Send_Senser();				//��ʾ����������
		break;
	case 3:
		Data_Send_MotoPWM();			//��ʾ�������
		break;
	case 4:
//		Data_Send_RCData();		//��ʾң��������
		break;
	case 5:
		if(_cnt_pid++>300)
		{
			_cnt_pid=0;
			Data_Send_PID1();	
		}		
		break;
	}	
	_cnt++;		
}

void Data_Receive_Anl(u8 *data_buf,u8 num)			//��Ƭ��������λ������
{
	u8 sum = 0;
	u8 i=0;
	
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
/*	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;			//�ж�֡ͷ*/

/*	if(*(data_buf+2)==0X10)								//�ж�pid
	{*/
			Roll_PID.Kp = (float)((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			Roll_PID.Ki = (float)((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			Roll_PID.Kd = (float)((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			Pitch_PID.Kp = (float)((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			Pitch_PID.Ki = (float)((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			Pitch_PID.Kd = (float)((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			Yaw_PID.Kp = (float)((u16)(*(data_buf+16)<<8)|*(data_buf+17));
			Yaw_PID.Ki = (float)((u16)(*(data_buf+18)<<8)|*(data_buf+19));
			Yaw_PID.Kd = (float)((u16)(*(data_buf+20)<<8)|*(data_buf+21));
			Data_Send_Check(sum);
//	}
}


// uint8_t zz=0;		//usart0��������   ��ʱ����
// //---------------------- �����ж� --------------------------------------
// void  UART0_Handler(void)
// {
// 	static uint8_t _cnt=0,Receieve_Data[32]={0},Seven_Cnt=1;
// 	zz = UARTCharGet(UART0_BASE);			//zz==���ڽ��յ���8λ����

// 	if(_cnt>=2 && Receieve_Data[1]!=0xAF)
// 	{
// 		_cnt=0;
// 		return;
// 	}		
// 	Receieve_Data[_cnt]=zz;
// 	_cnt++;
// 	if(_cnt==23)
// 	{
// 		Uart1_Send_Buf(Receieve_Data,_cnt);
// 		if(Seven_Cnt%7==1)
// 		{
// 			Data_Receive_Anl(Receieve_Data,_cnt);			//�ı�pid����
// 			Data_Send_PID1();			
// 		}
// 		else if(Seven_Cnt==7)		Seven_Cnt=0;
// 		Seven_Cnt++;
// 		_cnt=0;		
// 	}

// }   













