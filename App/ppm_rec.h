/***************************************************************************************************
  * 文 件 名: PPM_REC.h
  * 作    者：罗民
  *	日    期：2014/7/9
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
#ifndef __PPM_REC_H
#define __PPM_REC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* --------------头文件包含 ----------------------------------------------------------------------*/
#include "mygpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
/**************************************************************************************************/
/*--------------开关定义--------------------------------------------------------------------------*/
extern uint8_t flag;
/**************************************************************************************************/

/* -------------出口变量模型----------------------------------------------------------------------*/
typedef struct
{
	int16_t EXP_Throt;                 // 目标油门
	
	int16_t EXP_Roll;                  // 目标Roll
	int16_t EXP_Pitch;        
	int16_t EXP_Yaw;
	
	int8_t  NSWF;                         // 三档开关
	uint8_t AutoMode;                   // 摄像头域值调节旋钮
}
Control_Signal_Tpedef;
/**************************************************************************************************/	 
	 
/*--------------出口函数--------------------------------------------------------------------------*/
void PPM_Rec_Init(void);
void PPM_Data_Get(Control_Signal_Tpedef* con);
/**************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif 

/*****************************@版权 luomin*********************************************************/
