#ifndef __Main_H
#define __Main_H


#include "stdint.h"
#include "stdbool.h"

/* 文件夹：inc 	                  
 * 说  明：注意包含两类文件
 *	       -1  由芯片名字命名的文件，它主要包含此芯片寄存器的宏 
 *         -2  由外设名字命名的文件，它主要包含此外设寄存器位的宏
 *         -3  asmdefs.h，它主要包含一些汇编指令的宏
 *
 */
#include "inc/tm4c123gh6pz.h"	     //此类文件这里只包含这一个，其余的根据需要自行添加
//tm4c123gh6pz.h包含了以下文件所定义的大部分的宏
//#include "inc/hw_adc.h"
//#include "inc/hw_aes.h"
//#include "inc/hw_can.h"
//#include "inc/hw_ccm.h"
//#include "inc/hw_comp.h"
//#include "inc/hw_des.h"
//#include "inc/hw_eeprom.h"
//#include "inc/hw_emac.h"
//#include "inc/hw_epi.h"
//#include "inc/hw_fan.h"
//#include "inc/hw_flash.h"
#include "inc/hw_gpio.h"
//#include "inc/hw_hibernate.h"
//#include "inc/hw_i2c.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_lcd.h"
#include "inc/hw_memmap.h"			   //定义了一些外设寄存器地址
//#include "inc/hw_nvic.h"
//#include "inc/hw_pwm.h"
//#include "inc/hw_qei.h"
//#include "inc/hw_shamd5.h"
//#include "inc/hw_ssi.h"
//#include "inc/hw_sysctl.h"
//#include "inc/hw_sysexc.h"
// #include "inc/hw_timer.h"
#include "inc/hw_types.h"
//#include "inc/hw_uart.h"
//#include "inc/hw_udma.h"
//#include "inc/hw_usb.h"
//#include "inc/hw_watchdog.h"

#include "inc/asmdefs.h"             
/*================================================================================*/
/* 文件夹：lib 	                  
 * 说  明：这是Ti官方提供的库
 *		   如需调用，自行解除注释即可
 */
#include "driverlib/adc.h"
#include "driverlib/aes.h"	  
#include "driverlib/can.h"
#include "driverlib/comp.h"
#include "driverlib/cpu.h"                   //这个包含有问题，暂未解决，不能使用
#include "driverlib/crc.h"
#include "driverlib/debug.h"
#include "driverlib/des.h"
#include "driverlib/eeprom.h"
#include "driverlib/emac.h"
#include "driverlib/epi.h"
#include "driverlib/flash.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/lcd.h"
#include "driverlib/mpu.h"
//#include "driverlib/onewrir.h"	           //这个包含有问题，暂未解决，不能使用
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/rtos_bindings.h"
#include "driverlib/shamd5.h"
#include "driverlib/ssi.h"
#include "driverlib/sw_crc.h"
#include "driverlib/sysctl.h"				  
#include "driverlib/sysexc.h"
#include "driverlib/systick.h"		 
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/usb.h"
#include "driverlib/watchdog.h"

#include "selfPWM.h"

#include "ppm_rec.h"
#include "sysclock.h"
#include "mpu6050.h"
#include "angle_core.h"
#include "PID_Count.h"
#include "ultrasonic_Height.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h" 
#include "driverlib/pin_map.h"

#include "MyADC.h"
#include "Camera_Navigation.h"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#include "GpioInit.h"
#include "initialize.h"
#include "selfUart.h"
//--------------------------------------------------------------------------------
//   变量定义 
//--------------------------------------------------------------------------------
extern MPU6050_Typedef _6050_Init_Data;
extern volatile Angle_Typedef _Angle_Data;
extern uint32_t boot_timer;

extern uint32_t StartTime,PresentTime;


//-----------------------------------------------------------------------------------
//  自定义文件

void SendToSlave(void);
uint32_t GetSysTimer(void);

void SaveRCData(void);

void tim2_init(void);




#endif







