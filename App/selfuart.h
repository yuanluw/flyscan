/***************************************************************************************************
  * �� �� ��: selfuart.h
  * ��    �ߣ�����
  *	��    �ڣ�2014/7/6
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
#ifndef __SELFUART_H
#define __SELFUART_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

/* --------------ͷ�ļ����� ----------------------------------------------------------------------*/

//#include"main.h"
#include "inc_config.h"
#include "stdint.h"
#include "stdbool.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "inc/tm4c123gh6pz.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
	  
/*--------------���ض���--------------------------------------------------------------------------*/

/**************************************************************************************************/

/* -------------���ڱ���ģ��----------------------------------------------------------------------*/

/**************************************************************************************************/	 
	 
/*--------------���ں���--------------------------------------------------------------------------*/
void UART0_Conf(uint32_t Band);


	 
void Data_Send_Status(void);
void Data_Send_RCData(void);
void Data_Send_Senser(void);
void Data_Send_MotoPWM(void);
void Data_Send_Check(u16 check);
void Data_Send_PID2(void);
void Data_Send_PID1(void);
void Data_Receive_Anl(u8 *data_buf,u8 num);			//��Ƭ��������λ������
void Uart1_Send_Buf(uint8_t *buf,uint8_t len);
void Show_Data(void);
/**************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif 

/*****************************@��Ȩ luomin*********************************************************/
