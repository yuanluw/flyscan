#include "mygpio.h"
#include "inc/tm4c123gh6pz.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "stdint.h"
#include "stdbool.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"


void GPIO_Init(GPIO_InitTypeDef *GPIO_InitStruct)
{
	uint32_t GPIOx;
	if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOD)
		GPIOx =  0x40003000+(GPIO_InitStruct->GPIO_Port<<12);
	else
	if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOH)
		GPIOx =  0x40023000+((GPIO_InitStruct->GPIO_Port-(uint32_t)GPIOD)<<12); 
	else
	if(GPIO_InitStruct->GPIO_Port == GPIOJ)
		GPIOx = GPIO_PORTJ_BASE;
	if(GPIO_InitStruct->GPIO_Port == GPIOK)
		GPIOx = GPIO_PORTK_BASE;
	SysCtlPeripheralEnable((uint32_t)GPIO_InitStruct->GPIO_Port - (uint32_t)0x01 + SYSCTL_PERIPH_GPIOA);  //时钟使能

	GPIODirModeSet(GPIOx,GPIO_InitStruct->GPIO_Pin,GPIO_InitStruct->GPIO_Dir);	//GPIO方向设置

	GPIOPadConfigSet(GPIOx,GPIO_InitStruct->GPIO_Pin,
	                 GPIO_InitStruct->GPIO_Drive,GPIO_InitStruct->GPIO_Type);  //IO口驱动电流及类型设置
	
	if(GPIO_InitStruct->GPIO_IntSource == GPIO_INT_PIN) //判断是否为引脚中断
	{
		GPIOIntEnable((GPIOx),GPIO_InitStruct->GPIO_Pin);
		GPIOIntTypeSet(GPIOx,GPIO_InitStruct->GPIO_Pin,GPIO_InitStruct->GPIO_IntType); //IO中断类型
		if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOE)
			IntEnable(INT_GPIOA + GPIO_InitStruct->GPIO_Port-GPIOA); // 使能GPIOF端口中断
		else
		if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOH)
			IntEnable(INT_GPIOF + GPIO_InitStruct->GPIO_Port-GPIOF); // 使能GPIOF端口中断
		else
		if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOK)
			IntEnable(INT_GPIOJ + GPIO_InitStruct->GPIO_Port-GPIOJ); // 使能GPIOF端口中断
	}
	else
	{
		GPIOIntTypeSet(GPIOx,GPIO_InitStruct->GPIO_Pin,GPIO_InitStruct->GPIO_IntType); //IO中断类型
	}	
}

//----------------------------------------------------------------------------------------------------
//  & 函数名：  普通输出模式配置
//  & 参  数： GPIOx   GPIO_PIN_x
//  & 返  回： 无
//----------------------------------------------------------------------------------------------------
void OUT_NormalGPIO_Conf(uint8_t GPIOx,uint32_t GPIO_PIN_x)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Port 		= GPIOx;
	GPIO_InitStruct.GPIO_Pin  		= GPIO_PIN_x;
  GPIO_InitStruct.GPIO_Dir  		= GPIO_DIR_MODE_OUT;
	GPIO_InitStruct.GPIO_Drive  	= GPIO_STRENGTH_2MA;
  GPIO_InitStruct.GPIO_IntSource  = GPIO_INT_NONE;
	GPIO_InitStruct.GPIO_Type   	= GPIO_PIN_TYPE_STD;
	GPIO_Init(&GPIO_InitStruct);  
}
//----------------------------------------------------------------------------------------------------
//  & 函数名：  普通输入模式配置
//  & 参  数： GPIOx   GPIO_PIN_x
//  & 返  回： 无
//----------------------------------------------------------------------------------------------------
void IN_NormalGPIO_Conf(uint8_t GPIOx,uint32_t GPIO_PIN_x)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Port 		= GPIOx;
	GPIO_InitStruct.GPIO_Pin  		= GPIO_PIN_x;
  GPIO_InitStruct.GPIO_Dir  		= GPIO_DIR_MODE_IN;
	GPIO_InitStruct.GPIO_Drive  	= GPIO_STRENGTH_2MA;
  GPIO_InitStruct.GPIO_IntSource  = GPIO_INT_NONE;
	GPIO_InitStruct.GPIO_Type   	= GPIO_PIN_TYPE_OD;
	GPIO_Init(&GPIO_InitStruct);  
}
//----------------------------------------------------------------------------------------------------
//  & 函数名： 外部中断模式配置
//  & 参  数： GPIOx   GPIO_PIN_x   GPIOIntTypex
//  & 返  回： 无
//----------------------------------------------------------------------------------------------------
void EXTI_GPIO_Conf(uint8_t GPIOx,uint32_t GPIO_PIN_x,uint32_t GPIOIntTypex)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Port 		= GPIOx;
	GPIO_InitStruct.GPIO_Pin  		= GPIO_PIN_x;
  GPIO_InitStruct.GPIO_Dir  		= GPIO_DIR_MODE_IN;
	GPIO_InitStruct.GPIO_Drive  	= GPIO_STRENGTH_2MA;
  GPIO_InitStruct.GPIO_IntSource  = GPIO_INT_PIN;
	GPIO_InitStruct.GPIO_Type   	= GPIO_PIN_TYPE_OD;
	GPIO_InitStruct.GPIO_IntType  = GPIOIntTypex;
	GPIO_Init(&GPIO_InitStruct);  
}

