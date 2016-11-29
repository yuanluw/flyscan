/***************************************************************************************************
  * �� �� ��: selfPWM.h
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
	/* ------------��ֹ�ݹ�����Ķ���---------------------------------------------------------------*/
#ifndef __SELFPWM_H
#define __SELFPWM_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* --------------ͷ�ļ����� ----------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
/**************************************************************************************************/
/*--------------���ض���--------------------------------------------------------------------------*/
#define PWM_Max  1800//1600

/**************************************************************************************************/

/* -------------���ڱ���ģ��----------------------------------------------------------------------*/

/**************************************************************************************************/	 
	 
/*--------------���ں���--------------------------------------------------------------------------*/
void PWM_OutConf(void);
void Moter_Out_Set(uint16_t out_value[]);

/**************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif 

/*****************************@��Ȩ luomin*********************************************************/