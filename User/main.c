#include "inc_config.h"


extern uint8_t course_reserval;
extern uint8_t suspend_time;  //fly step
uint8_t flag;
volatile MPU6050_Typedef _6050_Data;
volatile Angle_Typedef _Angle_Data;
Control_Signal_Tpedef _PPM_Data;
uint16_t out[4]={0},Stop=0;
float outTemp[4];
float throcontemp ; 
#define Throt_TH            350
uint16_t ADTemp=0xf02;
uint8_t down_flag=0;
int8_t Xdirection=0,Ydirection=0,Down=0;
uint8_t Receieve_Data[10]={0},zz,Cnt=0,PPMtemp;
int16_t send_data[10]={0};
int main(void)
     {

			
	uint8_t i;
	uint8_t testco=0;
	SysInit();
	
	while(SysTimeOut_Flag==0){}
			SysTimeOut_Flag=0;		
	  
	while(1)
	{		
		Control_Mode = 0;
			PPM_Data_Get(&_PPM_Data);		
			Get_Angle(&_Angle_Data,&_6050_Data);

		if((_PPM_Data.NSWF==0x02) && (_PPM_Data.EXP_Throt>Throt_TH))
		{		

	
			Roll_PID.Error += (_Angle_Data.Roll+_PPM_Data.EXP_Roll/280.0f)*1.2f;
			Pitch_PID.Error += (-_Angle_Data.Pitch+_PPM_Data.EXP_Pitch/280.0f)*1.2f;
			//------------- PID 流程 -----------------------------------------------
			Roll_PID.Error += _6050_Data.Gyro_Data[0]*0.02;
			Pitch_PID.Error += _6050_Data.Gyro_Data[1]*0.02;
			Roll_PID.Error /=2;
			Pitch_PID.Error /=2;
			
			// 清除积分
			if(testco++>250)
			{		
				testco=0;
				Pitch_PID.PIDInter_Sum/=1.5f;
 				Roll_PID.PIDInter_Sum/=1.5f;
			}
			
			Yaw_PID.Error = -_6050_Data.Gyro_Data[2]*0.02+(_PPM_Data.EXP_Yaw/600.0f)*1.2f;       // 原 500
			Attitude_PID_Process();
			if(_PPM_Data.EXP_Yaw!=0) 
			{
				Yaw_PID.PIDInter_Sum=0;
				_Angle_Data.Yaw=0;
			}
			//-------------- 电机输出 ---------------------------------------------
			outTemp[0] = (_PPM_Data.EXP_Throt/12+(Roll_PID.PIDFin_Sum-Pitch_PID.PIDFin_Sum+Yaw_PID.PIDFin_Sum)*0.01f);
			outTemp[1] = (_PPM_Data.EXP_Throt/12+(Roll_PID.PIDFin_Sum+Pitch_PID.PIDFin_Sum-Yaw_PID.PIDFin_Sum)*0.01f);
			outTemp[2] = (_PPM_Data.EXP_Throt/12+(-Roll_PID.PIDFin_Sum-Pitch_PID.PIDFin_Sum-Yaw_PID.PIDFin_Sum)*0.01f);
			outTemp[3] = (_PPM_Data.EXP_Throt/12+(-Roll_PID.PIDFin_Sum+Pitch_PID.PIDFin_Sum+Yaw_PID.PIDFin_Sum)*0.01f);
			//----------- 最值限制 ------------------------------------------------
			for(i=0;i<4;i++)
			{
				if(outTemp[i] <1)  out[i] =1;
				else if(outTemp[i] >PWM_Max)  out[i] =PWM_Max;
				else out[i] = (uint16_t)outTemp[i];
			}
			RedLED_ON;
		
		}
		else if(_PPM_Data.NSWF==0x01)
		{
			flag=1;
			RedLED_OFF;
			SysDataInit();
			out[0] = 0;
			out[1] = 0;
			out[2] = 0;
			out[3] = 0;
		}
		else 
		{
			RedLED_ON;
			out[0] = 0;
			out[1] = 0;
			out[2] = 0;
			out[3] = 0;
			
		}

		
		while(SysTimeOut_Flag==0){}	//wait 4ms
			SysTimeOut_Flag=0;
			
		if(_Angle_Data.Roll>55 || _Angle_Data.Pitch>55 ||		//倾斜角度过大    停止转动
			_Angle_Data.Roll<-55 || _Angle_Data.Pitch<-55 ||Stop==1 )			
			{
				out[0] = 0;
				out[1] = 0;
				out[2] = 0;
				out[3] = 0;
				Stop=1;
			}
			
		Moter_Out_Set(out);
	}
}



void  UART0_Handler(void)
{
	zz = UARTCharGet(UART0_BASE);				  
	if(Cnt>=1 && Receieve_Data[0]!=0xAA  )
	{
		Cnt=0;
		return;
	}		
	if(Cnt>=2 &&  Receieve_Data[1]!=0xAF )
	{
		Cnt=0;
		return;
	}		
	Receieve_Data[Cnt]=zz;
	Cnt++;
	if(Cnt==6 && Receieve_Data[0]==0xAA )			//camera data
	{
		if(Receieve_Data[5]==0xab)	
		{
				Xdirection=Receieve_Data[2];
				Ydirection=Receieve_Data[3];
				down_flag=Receieve_Data[4];
			if(course_reserval)
				Down=1;
			//Down=Receieve_Data[4];
		}
		Cnt=0;		
	} 
	
}   
