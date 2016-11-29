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
	SysCtlPeripheralEnable((uint32_t)GPIO_InitStruct->GPIO_Port - (uint32_t)0x01 + SYSCTL_PERIPH_GPIOA);  //ʱ��ʹ��

	GPIODirModeSet(GPIOx,GPIO_InitStruct->GPIO_Pin,GPIO_InitStruct->GPIO_Dir);	//GPIO��������

	GPIOPadConfigSet(GPIOx,GPIO_InitStruct->GPIO_Pin,
	                 GPIO_InitStruct->GPIO_Drive,GPIO_InitStruct->GPIO_Type);  //IO��������������������
	
	if(GPIO_InitStruct->GPIO_IntSource == GPIO_INT_PIN) //�ж��Ƿ�Ϊ�����ж�
	{
		GPIOIntEnable((GPIOx),GPIO_InitStruct->GPIO_Pin);
		GPIOIntTypeSet(GPIOx,GPIO_InitStruct->GPIO_Pin,GPIO_InitStruct->GPIO_IntType); //IO�ж�����
		if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOE)
			IntEnable(INT_GPIOA + GPIO_InitStruct->GPIO_Port-GPIOA); // ʹ��GPIOF�˿��ж�
		else
		if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOH)
			IntEnable(INT_GPIOF + GPIO_InitStruct->GPIO_Port-GPIOF); // ʹ��GPIOF�˿��ж�
		else
		if(GPIO_InitStruct->GPIO_Port <= (uint32_t)GPIOK)
			IntEnable(INT_GPIOJ + GPIO_InitStruct->GPIO_Port-GPIOJ); // ʹ��GPIOF�˿��ж�
	}
	else
	{
		GPIOIntTypeSet(GPIOx,GPIO_InitStruct->GPIO_Pin,GPIO_InitStruct->GPIO_IntType); //IO�ж�����
	}	
}

//----------------------------------------------------------------------------------------------------
//  & ��������  ��ͨ���ģʽ����
//  & ��  ���� GPIOx   GPIO_PIN_x
//  & ��  �أ� ��
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
//  & ��������  ��ͨ����ģʽ����
//  & ��  ���� GPIOx   GPIO_PIN_x
//  & ��  �أ� ��
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
//  & �������� �ⲿ�ж�ģʽ����
//  & ��  ���� GPIOx   GPIO_PIN_x   GPIOIntTypex
//  & ��  �أ� ��
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

