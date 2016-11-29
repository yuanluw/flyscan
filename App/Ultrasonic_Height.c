#include "ultrasonic_Height.h"
#include "SYSCLOCK.h"
#include "PID_Count.h"
#include "driverlib/uart.h"
/**************************************************************************************************/
/*--------�����Ķ���-------------------------------------------------------------------------------*/
volatile uint32_t Ulatrasonic_Sta,Ulatrasonic_Count,Ulatrasonic_CountTemp;           // ���ݼ�¼
volatile uint8_t Ulacount_flag=0;                                   // ����ı�־λ

PID_Typedef Height_PID;

volatile float Height_PID_EXP;                         // PID��Ŀ��߶�ֵ

// ��ͣ�ı�������
#define HeightFlyM   2                  // 1 ��еʽ���

float rec_Suspension=0;                   //����ʱ�����ŵļ�¼
uint8_t stop=0;                            // ֹͣ״̬
uint8_t Step_FlySta=0;                   // ��ɽ���

float FloatThro = 5500;               // ��������
float temp;

uint8_t use_camera=0;
/**************************************************************************************************/
/*--------��  �Ķ���-------------------------------------------------------------------------------*/
#define      Height_Kp                        8000//7800//8000
#define      Height_Ki                        18//20//28//10//28     // 5 Ч������
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

float Filer_buff[4]={0};                   // �˲���������
uint8_t filterpos=0;                       // �˲����洢��λ��
float upfilter_data=0;                     // �ϴ��˲��Ľ��
#define MaxFilerStep               100//250     // �����˲�����  

volatile uint8_t UpDown=0;                                  // �����־ 

uint8_t WaitFly=0;               // �ȴ���ɵı�־λ

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
	
	//  �߶ȵ��˲������ݳ�ʼ��
	upfilter_data=0;
	Filer_buff[0]=0;
	Filer_buff[1]=0;
	Filer_buff[2]=0;
	Filer_buff[3]=0;
	filterpos=0;
	
	Ulacount_flag=0;
	
	rec_Suspension=0;
	stop=0;                            // ֹͣ״̬
	Step_FlySta=0;                   // ��ɽ���
	FloatThro = 5500;               // ��������
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

		//--------- �˲� ------------------------------
		Det_Temp = Ulatrasonic_Count*0.0106;    //0.01053088025769683f;   	
		//  ȥ������������Χ���ź�
		if(Det_Temp > 2100||Det_Temp <4) 
		{
			Ulacount_flag++;
			return;
		}
		
		Filer_buff[filterpos] = Det_Temp;
		// ȥ������ź�
		if(Filer_buff[filterpos]- upfilter_data>MaxFilerStep)                 
			Filer_buff[filterpos] = upfilter_data + MaxFilerStep;
		else if(Filer_buff[filterpos]- upfilter_data<-MaxFilerStep)
			Filer_buff[filterpos] = upfilter_data - MaxFilerStep;
		
		// ƽ��ֵ�˲�
		*Dis = Filer_buff[0]+Filer_buff[1]+Filer_buff[2]+Filer_buff[3];
		*Dis /=4;
		upfilter_data = *Dis;            // ��¼��ʷ
		filterpos++;                             // ѭ��
		if(filterpos > 3) filterpos=0;
	}
		Ulacount_flag++;
}




uint16_t HTimes_Count=0;          // �߶��ȶ���ʱ
uint16_t StopF=0;                     // ֹͣ�����������ļ�����
float Rema_thro=0;                        // ���ŵļ�¼ֵ

void Height_Pocess(float* thro,uint16_t adcv)
{


		// �߶Ȼ�ȡ
		Detection_distance(&temp);
		
	// ���ؼ�¼ֵ 
		if(Ulacount_flag!=1)  
		{
			*thro = Rema_thro;
			return;
		}
		
		//----- ��ʼ���� ----------------------
		if(UpDown == 0)
		{	
				//---------------- ֱ��������ͣ������� -------------------------------------------
				if(Step_FlySta==0)            // ֱ�����
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


					//---------  �߶�PID����  -----------------------------
					Height_PID.Error = Height_PID_EXP- temp;

					if(Height_PID.Error < 0) 
					{
						Height_PID.PIDInter_Sum -= Height_PID.Error*(Height_PID.Ki)*0.8f;  // ԭ0.9
					}
					PID_Count(&Height_PID);
					
					*thro = Height_PID.PIDFin_Sum/2500;                          // ���ĵ���������  	
				}

			//------- ������������
				*thro +=FloatThro;
				if(temp>1400)   	*thro=FloatThro*0.9f;		
				stop=0;                            // ֹͣʱ��Ҫ�õ�
				
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
				StopF=0;               // ��ռ���
			}
			else if(temp>100)
			{
				*thro = (Height_PID.PIDFin_Sum/2500 + +FloatThro)/0.9f;//1.3f;     
				StopF=0;               // ��ռ���
			}
			else 
			{
				*thro=0;
				stop=1;
			}
		//------- ֱ��ֹͣ
			if(stop) *thro=0;
		//------ �ָ���ɲ��� 
			Step_FlySta=0;

		}
		
		Rema_thro = *thro;
}

//------------- �жϺ��� ---------------------------------------------------------------------------
uint8_t ulaInt_Flag;
void  GPIOJ_Handler(void)
{
	Ulatrasonic_CountTemp =  TimerValueGet(TIMER3_BASE,TIMER_A);
	ulaInt_Flag = GPIOIntStatus(GPIO_PORTJ_BASE, true); // ��ȡ�ж�״̬
  GPIOIntClear(GPIO_PORTJ_BASE, ulaInt_Flag); // ����ж�״̬����Ҫ
	//-------------- ���� -------------------------------------------
	if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_2)==0x0)  
			Ulatrasonic_Count = Ulatrasonic_Sta- Ulatrasonic_CountTemp;
	Ulatrasonic_Sta = Ulatrasonic_CountTemp;
}
