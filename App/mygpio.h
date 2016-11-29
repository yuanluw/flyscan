#ifndef __MYGPIO_H
#define __MYGPIO_H

#include "stdint.h"
#include "stdbool.h"
#include "inc/hw_types.h"

#define GPIOA	 0x01
#define GPIOB	 0x02
#define GPIOC	 0x03
#define GPIOD	 0x04
#define GPIOE	 0x05
#define GPIOF	 0x06
#define GPIOG	 0x07
#define GPIOH	 0x08
#define GPIOJ	 0x09
#define GPIOK	 0x0A

//#define IO(GPIOx,pin)   HWREG(0x40003000+(GPIOx<<12)+(1<<(2+pin)))
#define IOA(pin) 	HWREG(GPIO_PORTA_BASE+(1<<(2+pin)))
#define IOB(pin) 	HWREG(GPIO_PORTB_BASE+(1<<(2+pin)))
#define IOC(pin) 	HWREG(GPIO_PORTC_BASE+(1<<(2+pin)))
#define IOD(pin) 	HWREG(GPIO_PORTD_BASE+(1<<(2+pin)))
#define IOE(pin) 	HWREG(GPIO_PORTE_BASE+(1<<(2+pin)))
#define IOF(pin) 	HWREG(GPIO_PORTF_BASE+(1<<(2+pin)))
#define IOG(pin) 	HWREG(GPIO_PORTG_BASE+(1<<(2+pin)))
#define IOH(pin) 	HWREG(GPIO_PORTH_BASE+(1<<(2+pin)))
#define IOJ(pin) 	HWREG(GPIO_PORTJ_BASE+(1<<(2+pin)))
#define IOK(pin) 	HWREG(GPIO_PORTK_BASE+(1<<(2+pin)))

#define GPIO_INT_PIN    0x00000200
#define GPIO_INT_NONE	0x00000000

typedef struct
{
  uint8_t  GPIO_Port;		                 /* GPIO组   GPIOA -- GPIOJ   */	

                                             									 
  uint16_t GPIO_Pin;                         /* 引脚     GPIO_PIN_0 -- GPIO_PIN_7  */                 


  uint32_t GPIO_Dir;						          /* 方向     xxx
                                                         (xxx: GPIO_DIR_MODE_IN / GPIO_DIR_MODE_OUT / GPIO_DIR_MODE_HW)  */

  uint32_t GPIO_IntType;    		         /* 中断     xxx
  											             (xxx: GPIO_FALLING_EDGE / GPIO_RISING_EDGE / GPIO_BOTH_EDGES
											                 / GPIO_LOW_LEVEL    / GPIO_HIGH_LEVEL  / GPIO_DISCRETE_INT  )  */

  uint32_t GPIO_Drive;  					 /* 驱动     GPIO_STRENGTH_xMA / GPIO_STRENGTH_8MA_SC(特殊)
                                                         (x: (2) / (4) / (6) / (8) / (10) / (12) )  */
  
  uint32_t GPIO_Type;                        /* 模式     xxx
  														 (xxx: GPIO_PIN_TYPE_STD / GPIO_PIN_TYPE_STD_WPU / GPIO_PIN_TYPE_STD_WPD
														     / GPIO_PIN_TYPE_OD  / GPIO_PIN_TYPE_ANALOG  / GPIO_PIN_TYPE_WAKE_HIGH
															 / GPIO_PIN_TYPE_WAKE_LOW )  */ 
															    
  uint32_t GPIO_IntSource;                   /* 中断     GPIO_INT_PIN / GPIO_INT_DMA  / GPIO_INT_NONE
                                                      								  */
														                              
}GPIO_InitTypeDef;


//--------------------- 输出模型 ----------------------------------------------------------------
void GPIO_Init(GPIO_InitTypeDef *GPIO_InitStruct);
void IN_NormalGPIO_Conf(uint8_t GPIOx,uint32_t GPIO_PIN_x);
void OUT_NormalGPIO_Conf(uint8_t GPIOx,uint32_t GPIO_PIN_x);
// GPIOx EXTI  配置之后  GPIOx  不能再利用上面的函数配置
void EXTI_GPIO_Conf(uint8_t GPIOx,uint32_t GPIO_PIN_x,uint32_t GPIOIntTypex);

//----------- example  ---------------------------------------------------------------------------
// if(IOG(0))
// 			IOF(6)=0x40;
// 		else
// 			IOF(6)=0x0;

#endif
