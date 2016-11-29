/***************************************************************************************************
  * 文 件 名: MYADC.h
  * 作    者：罗民
  *	日    期：2014/8/3
  * 版    本：V1.0
  * 开 发 板：TM4C123G6PZ--实验开发板
  * 修改时间：无 
  * 编 译 器: keil.4.54
  *-------------------------------------------------------------------------------------------------
  * 简    介:
  * 参    考:
  *	笔记名称:
  * 注    意：
  *************************************************************************************************/ 
	/* ------------防止递归包含的定义---------------------------------------------------------------*/
#ifndef __MYADC_H
#define __MYADC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* --------------头文件包含 ----------------------------------------------------------------------*/
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc_config.h"
/**************************************************************************************************/
/*--------------开关定义--------------------------------------------------------------------------*/

/**************************************************************************************************/

/* -------------出口变量模型----------------------------------------------------------------------*/

/**************************************************************************************************/	 
	 
/*--------------出口函数--------------------------------------------------------------------------*/
void ADC_Conf(void);
void ADC_GetV(uint16_t* data);
void ADC_Buff_Clear(void);
/**************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif 

/*****************************@版权 luomin*********************************************************/
