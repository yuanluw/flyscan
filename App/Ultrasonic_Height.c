#include "ultrasonic_Height.h"
#include "SYSCLOCK.h"
#include "PID_Count.h"
#include "driverlib/uart.h"
/**************************************************************************************************/
/*--------变量的定义-------------------------------------------------------------------------------*/
volatile uint32_t Ulatrasonic_Sta,Ulatrasonic_Count,Ulatrasonic_CountTemp;           // 数据记录
volatile uint8_t Ulacount_flag=0;                                   // 计算的标志位

PID_Typedef Height_PID;

volatile float Height_PID_EXP;                         // PID的目标高度值

// 悬停的变量定义
#define HeightFlyM   2                  // 1 机械式起飞

float rec_Suspension=0;                   //悬浮时的油门的记录
uint8_t stop=0;                            // 停止状态
uint8_t Step_FlySta=0;                   // 起飞节奏

float FloatThro = 5500;               // 悬浮油门
float temp;

uint8_t use_camera=0;
/**************************************************************************************************/
/*--------宏  的定义-------------------------------------------------------------------------------*/
#define      Height_Kp                        8000//7800//8000
#define      Height_Ki                        18//20//28//10//28     // 5 效果可以
#define      Height_Kd                        80000//74000//80000

void Ulatran_DataInit(void);

void Ulatran_Init(void)
{
	SysCtlPeripheralEnable(GPIOE);
	SysCtlPeripheralEnable(GPIOJ);
	EXTI_GPIO_Conf(GPIOJ,GPIO_PIN_2,GPIO_BOTH_EDGES);
	OUT_NormalGPIO_Conf(GPIOE,GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3,0x00);
	
	Ulatran_DataInit();
}

float Filer_buff[4]={0};                   // 滤波器缓冲区
uint8_t filterpos=0;                       // 滤波器存储的位置
float upfilter_data=0;                     // 上次滤波的结果
#define MaxFilerStep               100//250     // 最大的滤波阶梯  

volatile uint8_t UpDown=0;                                  // 起落标志 

uint8_t WaitFly=0;               // 等待起飞的标志位

void Ulatran_DataInit(void)
{
	Height_PID.Kp = Height_Kp;
	Height_PID.Ki = Height_Ki;
	Height_PID.Kd = Height_Kd;
	Height_PID.Error = 0;
	Height_PID.PreError = 0;
	Height_PID.PPreError = 0;
	Height_PID.PIDInter_Sum =0;
	Height_PID.PIDFin_Sum =0;
	
	Height_PID_EXP=0;
	
	//  高度的滤波器数据初始化
	upfilter_data=0;
	Filer_buff[0]=0;
	Filer_buff[1]=0;
	Filer_buff[2]=0;
	Filer_buff[3]=0;
	filterpos=0;
	
	Ulacount_flag=0;
	
	rec_Suspension=0;
	stop=0;                            // 停止状态
	Step_FlySta=0;                   // 起飞节奏
	FloatThro = 5500;               // 悬浮油门
	temp=0;
	
	Ulatrasonic_Sta=0;
	Ulatrasonic_Count=0;
	Ulatrasonic_CountTemp=0;
	
	UpDown=0;
	
	WaitFly=0;
}

void Detection_distance(volatile float* Dis)
{
	float Det_Temp;
	if(Ulacount_flag==1)
	{
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3,GPIO_PIN_3);
		delay_us(100);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3,0x00);
		Ulatrasonic_Sta =  TimerValueGet(TIMER3_BASE,TIMER_A);
	}
	else if(Ulacount_flag==3)
	{
		Ulacount_flag=0;

		//--------- 滤波 ------------------------------
		Det_Temp = Ulatrasonic_Count*0.0106;    //0.01053088025769683f;   	
		//  去掉超出正常范围的信号
		if(Det_Temp > 2100||Det_Temp <4) 
		{
			Ulacount_flag++;
			return;
		}
		
		Filer_buff[filterpos] = Det_Temp;
		// 去除冲击信号
		if(Filer_buff[filterpos]- upfilter_data>MaxFilerStep)                 
			Filer_buff[filterpos] = upfilter_data + MaxFilerStep;
		else if(Filer_buff[filterpos]- upfilter_data<-MaxFilerStep)
			Filer_buff[filterpos] = upfilter_data - MaxFilerStep;
		
		// 平均值滤波
		*Dis = Filer_buff[0]+Filer_buff[1]+Filer_buff[2]+Filer_buff[3];
		*Dis /=4;
		upfilter_data = *Dis;            // 记录历史
		filterpos++;                             // 循环
		if(filterpos > 3) filterpos=0;
	}
		Ulacount_flag++;
}




uint16_t HTimes_Count=0;          // 高度稳定计时
uint16_t StopF=0;                     // 停止螺旋桨动作的计数器
float Rema_thro=0;                        // 油门的记录值

void Height_Pocess(float* thro,uint16_t adcv)
{


		// 高度获取
		Detection_distance(&temp);
		
	// 返回记录值 
		if(Ulacount_flag!=1)  
		{
			*thro = Rema_thro;
			return;
		}
		
		//----- 开始计算 ----------------------
		if(UpDown == 0)
		{	
				//---------------- 直接利用悬停油门起飞 -------------------------------------------
				if(Step_FlySta==0)            // 直接起飞
				{

					if(temp>500) Step_FlySta = 1;

					FloatThro= -2.82f*adcv+21800;
					
					*thro = 500;//350;//300;
					
					
					if(adcv<300)
					{
						FloatThro =0;
					}						
				}
				else 
				{
					if(Step_FlySta==1)
					{
						if(temp>600) Step_FlySta=2;
						Height_PID_EXP = 600;
					}
					if(Step_FlySta==2)
					{
						if(temp>700) Step_FlySta=3;
						Height_PID_EXP = 700;
					}
					else if(Step_FlySta==3)
					{
						if(temp>800) Step_FlySta=4;
						Height_PID_EXP = 800;
					}
					else if(Step_FlySta==4)
					{
						if(temp>900) Step_FlySta=5;
						Height_PID_EXP = 900;
					}
					else if(Step_FlySta==5)
					{
					if(temp>1000) Step_FlySta=6;	
						Height_PID_EXP = 1000;
						
					}		
				
					else if(Step_FlySta==6)
					{		
						if(temp>1100) Step_FlySta=7; 
						Height_PID_EXP = 1100;	
					
					}
					else if(Step_FlySta==7)
					{		
						if(temp>1200) Step_FlySta=6; 
						Height_PID_EXP = 1150;

					}


					//---------  高度PID计算  -----------------------------
					Height_PID.Error = Height_PID_EXP- temp;

					if(Height_PID.Error < 0) 
					{
						Height_PID.PIDInter_Sum -= Height_PID.Error*(Height_PID.Ki)*0.8f;  // 原0.9
					}
					PID_Count(&Height_PID);
					
					*thro = Height_PID.PIDFin_Sum/2500;                          // 消耗的油门增量  	
				}

			//------- 加上悬浮油门
				*thro +=FloatThro;
				if(temp>1400)   	*thro=FloatThro*0.9f;		
				stop=0;                            // 停止时需要用的
				
		}
		else 
		{
			
			 if(temp>1000) 
				*thro = (Height_PID.PIDFin_Sum/2500 + +FloatThro)/1.4f;//1.3f;  
			else if(temp>800)
				*thro = (Height_PID.PIDFin_Sum/2500 + +FloatThro)/1.0f;//1.3f;  
			else if(temp>600)
				*thro = (Height_PID.PIDFin_Sum/2500 + +FloatThro)/1.4f;//1.0f;  
			else if(temp>400)
				*thro = (Height_PID.PIDFin_Sum/2500 + +FloatThro)/1.0f;//1.0f;  
			else if(temp>250)
			{
				*thro = (Height_PID.PIDFin_Sum/2500 + +FloatThro)/1.4f;//1.3f;     
				StopF=0;               // 清空计数
			}
			else if(temp>100)
			{
				*thro = (Height_PID.PIDFin_Sum/2500 + +FloatThro)/0.9f;//1.3f;     
				StopF=0;               // 清空计数
			}
			else 
			{
				*thro=0;
				stop=1;
			}
		//------- 直接停止
			if(stop) *thro=0;
		//------ 恢复起飞步骤 
			Step_FlySta=0;

		}
		
		Rema_thro = *thro;
}

//------------- 中断函数 ---------------------------------------------------------------------------
uint8_t ulaInt_Flag;
void  GPIOJ_Handler(void)
{
	Ulatrasonic_CountTemp =  TimerValueGet(TIMER3_BASE,TIMER_A);
	ulaInt_Flag = GPIOIntStatus(GPIO_PORTJ_BASE, true); // 读取中断状态
  GPIOIntClear(GPIO_PORTJ_BASE, ulaInt_Flag); // 清除中断状态，重要
	//-------------- 计算 -------------------------------------------
	if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_2)==0x0)  
			Ulatrasonic_Count = Ulatrasonic_Sta- Ulatrasonic_CountTemp;
	Ulatrasonic_Sta = Ulatrasonic_CountTemp;
}
