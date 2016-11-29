#ifndef __GPIOINIT_H
#define __GPIOINIT_H

#include "inc_config.h"

//------------- LED 宏定义 -----------------------------------------------------
//------------- LED 宏定义 -----------------------------------------------------
#define    WhiteLED_ON                ROM_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5,0x00)
#define    WhiteLED_OFF               ROM_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5,GPIO_PIN_5)

#define    RedLED_ON                  ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0,0x00)
#define    RedLED_OFF                 ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0,GPIO_PIN_0)

#define    Electromagnet_ON             ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1)
#define    Electromagnet_OFF            ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00)

#define			UseCamera							ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0,GPIO_PIN_0)
#define			NoCamera							ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0,0x00)

void LED_Electromagnet_Init(void);



#endif







