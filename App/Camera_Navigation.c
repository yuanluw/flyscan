/***************************************************************************************************
  * 文 件 名: Camera_Navigation.c
  * 作    者：罗民
  *	日    期：2014/8/14
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
/*--------包含头文件------------------------------------------------------------------------------*/
#include "Camera_Navigation.h"
/**************************************************************************************************/
/*--------变量的定义-------------------------------------------------------------------------------*/
uint8_t Nav_Step;                             // 导航进行的步骤
int8_t NavTempofC[3];
extern uint8_t UpDown;                         // 导航结束
uint8_t XuanTingD=0;                           // 悬停指示的节奏

PID_Typedef ZuoYouPID;
/**************************************************************************************************/
/*--------宏定义-------------------------------------------------------------------------------*/
#define       ZuoYouPID_Kp                300//100//300//450//300//400//450
#define       ZuoYouPID_Ki                0//8
#define       ZuoYouPID_Kd                0

/**************************************************************************************************/
/*-------进口函数----------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------函数模型----------------------------------------------------------------------------------*/
/***************************************************************************************************
	函数名: CmNa_Init
	输  入: void
	输  出:	void
	功  能:	导航数据初始化
****************************************************************************************************/	
void CmNa_Init(void)
{
	Nav_Step=100;                   // 默认摄像头不动作
	NavTempofC[0] =0;
	NavTempofC[1] =0;
	NavTempofC[2] =0;
	
	XuanTingD=0;
	
	//  PID 数据初始化
	ZuoYouPID.Kp = ZuoYouPID_Kp;
	ZuoYouPID.Ki = ZuoYouPID_Ki;
	ZuoYouPID.Kd = ZuoYouPID_Kd;
	ZuoYouPID.Error = 0;
	ZuoYouPID.PreError = 0;
	ZuoYouPID.PPreError = 0;
	ZuoYouPID.PIDInter_Sum =0;
	ZuoYouPID.PIDFin_Sum =0;
}
/***************************************************************************************************
	函数名: CmNa_Doing
	输  入: 
	输  出:	void
	功  能:	导航执行
****************************************************************************************************/	
uint8_t CmNaCount=0;                           // 摄像头的次数记录
uint8_t newvou=0;
extern uint8_t MainStep_Fly;

void CmNa_Doing(signed char* adata,Control_Signal_Tpedef* exp)
{
	int8_t FW;
	
	//---------- LED 指示悬停  --------------------------------------
	if(XuanTingD==1)
	{
		if(adata[0]&0xC0)
			ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0,0x00);                 // 白灯亮
		else if(adata[0]&0xc0 == 0xc0)
			XuanTingD=2;
	}
	else if(XuanTingD==2)
		ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0,GPIO_PIN_0);                 // 白灯灭
	
	adata[0] &= 0x3f;
	//   记录摄像头指令
		NavTempofC[0] = adata[0];
		NavTempofC[1] = adata[1];
		NavTempofC[2] = adata[2];

	// 导航节奏
// 	if(Nav_Step<4)   
// 	{
// 		//    做出判断
// 		if((NavTempofC[2]==30)&&(NavTempofC[1]==30))        // 整片区域全白
// 		{
// 			FW = -15;//20;//30;                              // 后退
// 		}
// 		else if((NavTempofC[1]==70)||(NavTempofC[2] ==70))              // 开始降落
// 		{
// 			FW = 0;
// 			//if(NavTempofC[0] > 15 && NavTempofC[0] < 45)
// 			if(newvou>100)
// 			{
// 				UpDown=1;               // 降落的第一步
// // 				FW = -30;
// // 				CmNaCount++;
// // 				if(CmNaCount>10)               // 判断其是否稳定  锁定在降落点
// // 				{
// // 					CmNaCount=0;
// // 					Nav_Step =3;
// // 				}
// 				// ----------- 已经找到目标  可以降落 ---------------------------------
// 				if(Nav_Step==3)  Nav_Step=4;
//  			}
// 		}
// 		else
// 		{
// 			FW = 40;//35;//27;
// 		}
// 	}
	
	if(MainStep_Fly==1)
	{
		//------- 换一种  ------------------------------------------------
		if(Nav_Step==0)                                           
		{
			if(XuanTingD!=1)   XuanTingD=1;            // 悬停指示改变
		//    做出判断
			if(NavTempofC[2]==30&&NavTempofC[1]==30)        // 整片区域全白
			{
				FW = 30;//20;                              // 前进
			}
			else if(NavTempofC[2] <20)
			{
				FW = 20- NavTempofC[2];
			}
			else if(NavTempofC[2] >55)              // 后退
			{
				FW = NavTempofC[2]-82;
			}
			else FW =0;
			
			CmNaCount=0;
		}
		//  找到黑色区域   再执行转变工作
		else if(Nav_Step ==1)
		{
			if(NavTempofC[2]==30&&NavTempofC[1]==30)
				FW = 30;
			else
			{
				FW=10;
				Nav_Step=2;
			}
			CmNaCount=0;
			newvou=0;
		}
		else if(Nav_Step ==2||Nav_Step ==3)
		{
			newvou++;
			if(newvou>240) newvou=240;
			//  搜索前进
			if((NavTempofC[2]==30)&&(NavTempofC[1]==30))        // 整片区域全白
				{
					FW = -15;//20;//30;                              // 后退
				}
				else if((NavTempofC[1]==70)||(NavTempofC[2] ==70))              // 开始降落
				{
					FW = 0;
					if(newvou>100)
					{
						UpDown=1;               // 降落的第一步
						// ----------- 已经找到目标  可以降落 ---------------------------------
					}
				}
				else
				{
					FW = 35;//40;//35;//27;
				}
		}
	}
	else if(MainStep_Fly==3)
	{
		//------- 换一种  ------------------------------------------------
		if(Nav_Step==0)                                           
		{
			if(XuanTingD!=1)   XuanTingD=1;            // 悬停指示改变
		//    做出判断
			if(NavTempofC[2]==30&&NavTempofC[1]==30)        // 整片区域全白
			{
				FW = -50;                             // 前进
			}
			else if(NavTempofC[1] <20)
			{
				FW = NavTempofC[1]-30;
			}
			else if(NavTempofC[1] >55)              // 后退
			{
				FW = 0;//82-NavTempofC[1];
			}
			else FW =0;
			
			CmNaCount=0;
		}
		//  找到黑色区域   再执行转变工作
		else if(Nav_Step ==1)
		{
			
				Nav_Step=2;
			CmNaCount=0;
			newvou=0;
		}
		else if(Nav_Step ==2||Nav_Step ==3)
		{
			newvou++;
			if(newvou>240) newvou=240;
			//  搜索前进
			if((NavTempofC[2]==30)&&(NavTempofC[1]==30))        // 整片区域全白
				{
					FW = 15;//20;//30;                              // 后退
				}
				else if((NavTempofC[1]==70)||(NavTempofC[2] ==70))              // 开始降落
				{
					FW = 0;
					if(newvou>100)
					{
						UpDown=1;               // 降落的第一步
						// ----------- 已经找到目标  可以降落 ---------------------------------
					}
				}
				else
				{
					FW = -59;
				}
		}
	}

	
	//========= 决策输出  ===================================================
	if(Nav_Step <10)
	{	
		//---------- 防止数据错误  ------------------------------------------
		if(FW >55||FW <-60) FW=0;
		if(NavTempofC[0] >60||NavTempofC[0]<0) NavTempofC[0]=30;
		//------ 控制前后  --------------------------------------------------
		exp->EXP_Roll += FW*180;//240;//220;//180;//200;//240;
		//------ 控制左右  ---------------------------------------------------
		ZuoYouPID.Error = NavTempofC[0];
		ZuoYouPID.Error =30 -ZuoYouPID.Error;
		
// 		if(ZuoYouPID.Error>0 && ZuoYouPID.PIDInter_Sum<0) ZuoYouPID.PIDInter_Sum=0;
// 		else if(ZuoYouPID.Error<0 && ZuoYouPID.PIDInter_Sum>0) ZuoYouPID.PIDInter_Sum=0;
		
// 		if(ZuoYouPID.Error<20&&ZuoYouPID.Error>-20)  ZuoYouPID.Error=0;
		PID_Count(&ZuoYouPID);
		// 执行决策
		exp->EXP_Pitch -= ZuoYouPID.PIDFin_Sum;
	}
}

/*****************************@版权 luomin*********************************************************/
