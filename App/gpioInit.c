#include "GpioInit.h"


//-------------- LED及电磁铁的初始化 --------------------------------------------
void LED_Electromagnet_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_5);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_4);
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_5);
	
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_1);
	
	WhiteLED_OFF;
	RedLED_OFF;
}
























