/***************************************************************************************************
  * �� �� ��: MYADC.h
  * ��    �ߣ�����
  *	��    �ڣ�2014/8/3
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
	/* ------------��ֹ�ݹ�����Ķ���---------------------------------------------------------------*/
#ifndef __MYADC_H
#define __MYADC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* --------------ͷ�ļ����� ----------------------------------------------------------------------*/
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc_config.h"
/**************************************************************************************************/
/*--------------���ض���--------------------------------------------------------------------------*/

/**************************************************************************************************/

/* -------------���ڱ���ģ��----------------------------------------------------------------------*/

/**************************************************************************************************/	 
	 
/*--------------���ں���--------------------------------------------------------------------------*/
void ADC_Conf(void);
void ADC_GetV(uint16_t* data);
void ADC_Buff_Clear(void);
/**************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif 

/*****************************@��Ȩ luomin*********************************************************/