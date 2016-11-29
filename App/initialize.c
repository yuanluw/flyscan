#include "initialize.h"


void SysInit(void)
{
	SysClock_Init();
	delay_ms(2);
	PWM_OutConf();
	delay_ms(500);

	UART0_Conf(115200);
	delay_ms(500);
	LED_Electromagnet_Init();
	RedLED_ON;
	PPM_Rec_Init();
	Ulatran_Init();
	
	delay_ms(500);
	WhiteLED_OFF;
	ADC_Conf();
 	IntEnable(INT_UART0);
	IntMasterEnable();
	SysTimeOut_Flag=0;
	SysDataInit();
}


extern volatile MPU6050_Typedef _6050_Data;
void SysDataInit(void)
{
	MPU6050_Init();
	// 在6050init之后
	
	Get_MPU6050_firstData(&_6050_Data);
	
 	Position_DataInit(_6050_Data.Acc_Data[2]);
// 	Position_DataInit(0x413e);
	
	PID_Init();
	//------- 超声波测量初始化  ---------------------------
	Ulatran_DataInit();
	
	ADC_Buff_Clear();
	
//	CmNa_Init();
	//---------- 数据初始化 --------------------------------
	_Angle_Data.Roll=0;
	_Angle_Data.Pitch=0;
	_Angle_Data.Yaw=0;

}

u8	Control_Mode=0;
void Mode_Det(void)
{
	uint8_t i=0;
	for(i=0;i<5;)
	{
		ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4,GPIO_PIN_4);
		delay_us(1);
		if(ROM_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_5))
			i++;
		else break;
		ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4,0x00);
		delay_us(1);
		if(ROM_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_5))
			break;
		else i++;
	}
	
	if(i<5) Control_Mode=0;              // 遥控器模式
	else Control_Mode=1;                   // 自主模式
	
	
// 	for(i=0;i<5;)
// 	{
// 		ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0,GPIO_PIN_0);
// 		delay_us(1);
// 		if(ROM_GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1))
// 			i++;
// 		else break;
// 		ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0,0x00);
// 		delay_us(1);
// 		if(ROM_GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1))
// 			break;
// 		else i++;
// 	}
	
// 	if(mii>=5) Debug_Auto=1;              // 遥控器模式 测试自主
// 	else Debug_Auto=0;                      // 不进行自主
}









