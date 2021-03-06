/***************************************************************************************************
  * 文 件 名: ultrasonic_Height.h
  * 作    者：罗民
  *	日    期：2014/7/15
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
#ifndef __ULTRASONIC_HEIGHT_H
#define __ULTRASONIC_HEIGHT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* --------------头文件包含 ----------------------------------------------------------------------*/
#include "inc_config.h"
/**************************************************************************************************/
/*--------------开关定义--------------------------------------------------------------------------*/

/**************************************************************************************************/

/* -------------出口变量模型----------------------------------------------------------------------*/

/**************************************************************************************************/	 
	 
/*--------------出口函数--------------------------------------------------------------------------*/
void Detection_distance(volatile float* Dis);
void Ulatran_Init(void);
void Height_Pocess(float* thro,uint16_t adcv);
void Ulatran_DataInit(void);
/**************************************************************************************************/
extern volatile uint8_t UpDown;   
extern float temp;
extern uint16_t HTimes_Count;
extern uint8_t use_camera;
#ifdef __cplusplus
}
#endif

#endif 

/*****************************@版权 luomin*********************************************************/
