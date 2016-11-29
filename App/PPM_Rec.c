/***************************************************************************************************
  * �� �� ��: PPM_REC.c
  * ��    �ߣ�����
  *	��    �ڣ�2014/7/9
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
#include "PPM_REC.h"
/**************************************************************************************************/
/*--------�����Ķ���-------------------------------------------------------------------------------*/

uint8_t ppmstasus1,ppmstasus2;                                              // �ж�״̬��ʱ��¼����
uint32_t PPM_Precount[6],PPM_Ncount[6];                                     // ���ݼ�¼
uint32_t PPM_Temp1,PPM_Temp2;                                               // ������ʱ����
/**************************************************************************************************/
/*-------- �궨�� -------------------------------------------------------------------------------*/
#define MinPPM               0x3F0E                    // ʵ��0x3F0F
#define MaxPPM               0x7F10                    // ʵ��0x7F0F
#define MidPPM               0x5F0F

#define Yaw_ppm_th           0x1000//0x900

/**************************************************************************************************/
/*-------���ں���----------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------����ģ��----------------------------------------------------------------------------------*/
/***************************************************************************************************
	������: PPM_Rec_Init
	��  ��: void
	��  ��:	void
	��  ��:	
****************************************************************************************************/	
void PPM_Rec_Init(void)
{
 	EXTI_GPIO_Conf(GPIOB,GPIO_PIN_2|GPIO_PIN_3,GPIO_BOTH_EDGES);
	EXTI_GPIO_Conf(GPIOG,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_BOTH_EDGES);
	
	//--------- ��ʱ����ʼ�� --------------------------------------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	TimerConfigure(TIMER3_BASE, TIMER_CFG_A_PERIODIC);             // ������  ���¼���
	TimerLoadSet(TIMER3_BASE, TIMER_A, 0xffffffff);
	TimerEnable(TIMER3_BASE, TIMER_A);
}

/***************************************************************************************************
	������: PPM_Data_Get
	��  ��: con
	��  ��:	void
	��  ��:	
****************************************************************************************************/	
void PPM_Data_Get(Control_Signal_Tpedef* con)
{
	con->EXP_Throt = PPM_Ncount[2] - MinPPM;  // ����
//	
	con->EXP_Roll = -(PPM_Ncount[0] - MidPPM);    
	con->EXP_Pitch = PPM_Ncount[1] - MidPPM; 
	con->EXP_Yaw = PPM_Ncount[3] - MidPPM; 
	
	if((con->EXP_Yaw <Yaw_ppm_th)&&(con->EXP_Yaw >-Yaw_ppm_th))  con->EXP_Yaw=0;
	
	if(PPM_Ncount[4] <MinPPM+0x1500)  con->NSWF = 3;
	else  if(PPM_Ncount[4] <MidPPM+0x1500)  con->NSWF = 2;
	else con->NSWF = 1;
		
	if(con->NSWF==1)
	{
			if(PPM_Ncount[2]<=MinPPM)
			{
							flag=0;
			}	
	}
	
	
	
	if(PPM_Ncount[5]>MidPPM)	con->AutoMode =0;
	else	con->AutoMode =1;
}

void  GPIOB_Handler(void)
{
	PPM_Temp1 =  TimerValueGet(TIMER3_BASE,TIMER_A);
	ppmstasus1 = GPIOIntStatus(GPIO_PORTB_BASE, true); // ��ȡ�ж�״̬
  GPIOIntClear(GPIO_PORTB_BASE, ppmstasus1); // ����ж�״̬����Ҫ
	if(ppmstasus1&GPIO_PIN_2)
	{
		if(GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_2)==0x0)  
			PPM_Ncount[0] = PPM_Precount[0] -PPM_Temp1;
		PPM_Precount[0] = PPM_Temp1;
	}
	if(ppmstasus1&GPIO_PIN_3)
	{
		if(GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_3)==0x0)  
			PPM_Ncount[1] = PPM_Precount[1] -PPM_Temp1;
		PPM_Precount[1] = PPM_Temp1;
	}
} 

void  GPIOG_Handler(void)
{
	PPM_Temp2 =  TimerValueGet(TIMER3_BASE,TIMER_A);
	ppmstasus2 = GPIOIntStatus(GPIO_PORTG_BASE, true); // ��ȡ�ж�״̬
  GPIOIntClear(GPIO_PORTG_BASE, ppmstasus2); // ����ж�״̬����Ҫ
	if(ppmstasus2&GPIO_PIN_0)
	{
		if(GPIOPinRead(GPIO_PORTG_BASE,GPIO_PIN_0)==0x0)  
			PPM_Ncount[2] = PPM_Precount[2] -PPM_Temp2;
		PPM_Precount[2] = PPM_Temp2;
	}
	if(ppmstasus2&GPIO_PIN_1)
	{
		if(GPIOPinRead(GPIO_PORTG_BASE,GPIO_PIN_1)==0x0)
			PPM_Ncount[3] = PPM_Precount[3] -PPM_Temp2;
		PPM_Precount[3] = PPM_Temp2;
	}
	if(ppmstasus2&GPIO_PIN_2)
	{
		if(GPIOPinRead(GPIO_PORTG_BASE,GPIO_PIN_2)==0x0)
			PPM_Ncount[4] = PPM_Precount[4] -PPM_Temp2;
		PPM_Precount[4] = PPM_Temp2;
	}
	if(ppmstasus2&GPIO_PIN_3)
	{
		if(GPIOPinRead(GPIO_PORTG_BASE,GPIO_PIN_3)==0x0)
			PPM_Ncount[5] = PPM_Precount[5] -PPM_Temp2;
		PPM_Precount[5] = PPM_Temp2;
	}
}

/*****************************@��Ȩ luomin*********************************************************/
