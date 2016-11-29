/***************************************************************************************************
  * �� �� ��: selfPWM.c
  * ��    �ߣ�����
  *	��    �ڣ�2014/7/7
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
#include "selfPWM.h"
/**************************************************************************************************/
/*--------�����Ķ���-------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------�ṹ��Ķ���-------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*-------���ں���----------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------����ģ��----------------------------------------------------------------------------------*/
/***************************************************************************************************
	������: PWM_OutConf
	��  ��: void
	��  ��:	int
	��  ��:	
****************************************************************************************************/	
void PWM_OutConf(void)
{
	//------------ ʱ������ -----------------------------------------
	SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	//----------- �������� ------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    GPIOPinConfigure(GPIO_PH2_M0PWM2);
		GPIOPinConfigure(GPIO_PH3_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_2);
		GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_3); 
		PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,4000 );
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 2000);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 2000);
		PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT|PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	//----------- �������� 2------------------------------------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
		GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
		GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5); 
		PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3,4000 );
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 2000);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 2000);
		PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT|PWM_OUT_7_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

uint8_t MoterFang=0;
void Moter_Out_Set(uint16_t out_value[])
{
	uint8_t mi;
	for(mi=0;mi<4;mi++)
		if(out_value[mi]!=0) out_value[mi]+=400;
// 	switch(MoterFang)
// 	{
// 		case 0:
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, out_value[2]+2000);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, out_value[0]+2000);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, out_value[1]+2000);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, out_value[3]+2000);
//  		MoterFang ++;break;
// 		case 1:
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, out_value[2]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, out_value[3]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, out_value[0]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, out_value[1]+2000);
//  		MoterFang ++;break;
// 		case 2:
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, out_value[1]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, out_value[3]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, out_value[2]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, out_value[0]+2000);
//  		MoterFang ++;break;
// 		case 3:
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, out_value[3]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, out_value[2]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, out_value[0]+2000);
// 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, out_value[1]+2000);
//  		MoterFang =0;break;
// 	}
}

/*****************************@��Ȩ luomin*********************************************************/
