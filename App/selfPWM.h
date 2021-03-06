/***************************************************************************************************
  * 文 件 名: selfPWM.h
  * 作    者：罗民
  *	日    期：2014/7/7
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
#ifndef __SELFPWM_H
#define __SELFPWM_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* --------------头文件包含 ----------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
/**************************************************************************************************/
/*--------------开关定义--------------------------------------------------------------------------*/
#define PWM_Max  1800//1600

/**************************************************************************************************/

/* -------------出口变量模型----------------------------------------------------------------------*/

/**************************************************************************************************/	 
	 
/*--------------出口函数--------------------------------------------------------------------------*/
void PWM_OutConf(void);
void Moter_Out_Set(uint16_t out_value[]);

/**************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif 

/*****************************@版权 luomin*********************************************************/
