/***************************************************************************************************
  * �� �� ��: Camera_Navigation.c
  * ��    �ߣ�����
  *	��    �ڣ�2014/8/14
  * ��    ����V1.0
  * �� �� �壺TM4C123G6PZ--ʵ�鿪����
  * �޸�ʱ�䣺�� 
  * �� �� ��: keil.4.54
  *-------------------------------------------------------------------------------------------------
  * ��    ��:
  * ��    ��:
  *	�ʼ�����:
  * ע    �⣺
  *************************************************************************************************/  
/*--------����ͷ�ļ�------------------------------------------------------------------------------*/
#include "Camera_Navigation.h"
/**************************************************************************************************/
/*--------�����Ķ���-------------------------------------------------------------------------------*/
uint8_t Nav_Step;                             // �������еĲ���
int8_t NavTempofC[3];
extern uint8_t UpDown;                         // ��������
uint8_t XuanTingD=0;                           // ��ָͣʾ�Ľ���

PID_Typedef ZuoYouPID;
/**************************************************************************************************/
/*--------�궨��-------------------------------------------------------------------------------*/
#define       ZuoYouPID_Kp                300//100//300//450//300//400//450
#define       ZuoYouPID_Ki                0//8
#define       ZuoYouPID_Kd                0

/**************************************************************************************************/
/*-------���ں���----------------------------------------------------------------------------------*/

/**************************************************************************************************/
/*--------����ģ��----------------------------------------------------------------------------------*/
/***************************************************************************************************
	������: CmNa_Init
	��  ��: void
	��  ��:	void
	��  ��:	�������ݳ�ʼ��
****************************************************************************************************/	
void CmNa_Init(void)
{
	Nav_Step=100;                   // Ĭ������ͷ������
	NavTempofC[0] =0;
	NavTempofC[1] =0;
	NavTempofC[2] =0;
	
	XuanTingD=0;
	
	//  PID ���ݳ�ʼ��
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
	������: CmNa_Doing
	��  ��: 
	��  ��:	void
	��  ��:	����ִ��
****************************************************************************************************/	
uint8_t CmNaCount=0;                           // ����ͷ�Ĵ�����¼
uint8_t newvou=0;
extern uint8_t MainStep_Fly;

void CmNa_Doing(signed char* adata,Control_Signal_Tpedef* exp)
{
	int8_t FW;
	
	//---------- LED ָʾ��ͣ  --------------------------------------
	if(XuanTingD==1)
	{
		if(adata[0]&0xC0)
			ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0,0x00);                 // �׵���
		else if(adata[0]&0xc0 == 0xc0)
			XuanTingD=2;
	}
	else if(XuanTingD==2)
		ROM_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0,GPIO_PIN_0);                 // �׵���
	
	adata[0] &= 0x3f;
	//   ��¼����ͷָ��
		NavTempofC[0] = adata[0];
		NavTempofC[1] = adata[1];
		NavTempofC[2] = adata[2];

	// ��������
// 	if(Nav_Step<4)   
// 	{
// 		//    �����ж�
// 		if((NavTempofC[2]==30)&&(NavTempofC[1]==30))        // ��Ƭ����ȫ��
// 		{
// 			FW = -15;//20;//30;                              // ����
// 		}
// 		else if((NavTempofC[1]==70)||(NavTempofC[2] ==70))              // ��ʼ����
// 		{
// 			FW = 0;
// 			//if(NavTempofC[0] > 15 && NavTempofC[0] < 45)
// 			if(newvou>100)
// 			{
// 				UpDown=1;               // ����ĵ�һ��
// // 				FW = -30;
// // 				CmNaCount++;
// // 				if(CmNaCount>10)               // �ж����Ƿ��ȶ�  �����ڽ����
// // 				{
// // 					CmNaCount=0;
// // 					Nav_Step =3;
// // 				}
// 				// ----------- �Ѿ��ҵ�Ŀ��  ���Խ��� ---------------------------------
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
		//------- ��һ��  ------------------------------------------------
		if(Nav_Step==0)                                           
		{
			if(XuanTingD!=1)   XuanTingD=1;            // ��ָͣʾ�ı�
		//    �����ж�
			if(NavTempofC[2]==30&&NavTempofC[1]==30)        // ��Ƭ����ȫ��
			{
				FW = 30;//20;                              // ǰ��
			}
			else if(NavTempofC[2] <20)
			{
				FW = 20- NavTempofC[2];
			}
			else if(NavTempofC[2] >55)              // ����
			{
				FW = NavTempofC[2]-82;
			}
			else FW =0;
			
			CmNaCount=0;
		}
		//  �ҵ���ɫ����   ��ִ��ת�乤��
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
			//  ����ǰ��
			if((NavTempofC[2]==30)&&(NavTempofC[1]==30))        // ��Ƭ����ȫ��
				{
					FW = -15;//20;//30;                              // ����
				}
				else if((NavTempofC[1]==70)||(NavTempofC[2] ==70))              // ��ʼ����
				{
					FW = 0;
					if(newvou>100)
					{
						UpDown=1;               // ����ĵ�һ��
						// ----------- �Ѿ��ҵ�Ŀ��  ���Խ��� ---------------------------------
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
		//------- ��һ��  ------------------------------------------------
		if(Nav_Step==0)                                           
		{
			if(XuanTingD!=1)   XuanTingD=1;            // ��ָͣʾ�ı�
		//    �����ж�
			if(NavTempofC[2]==30&&NavTempofC[1]==30)        // ��Ƭ����ȫ��
			{
				FW = -50;                             // ǰ��
			}
			else if(NavTempofC[1] <20)
			{
				FW = NavTempofC[1]-30;
			}
			else if(NavTempofC[1] >55)              // ����
			{
				FW = 0;//82-NavTempofC[1];
			}
			else FW =0;
			
			CmNaCount=0;
		}
		//  �ҵ���ɫ����   ��ִ��ת�乤��
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
			//  ����ǰ��
			if((NavTempofC[2]==30)&&(NavTempofC[1]==30))        // ��Ƭ����ȫ��
				{
					FW = 15;//20;//30;                              // ����
				}
				else if((NavTempofC[1]==70)||(NavTempofC[2] ==70))              // ��ʼ����
				{
					FW = 0;
					if(newvou>100)
					{
						UpDown=1;               // ����ĵ�һ��
						// ----------- �Ѿ��ҵ�Ŀ��  ���Խ��� ---------------------------------
					}
				}
				else
				{
					FW = -59;
				}
		}
	}

	
	//========= �������  ===================================================
	if(Nav_Step <10)
	{	
		//---------- ��ֹ���ݴ���  ------------------------------------------
		if(FW >55||FW <-60) FW=0;
		if(NavTempofC[0] >60||NavTempofC[0]<0) NavTempofC[0]=30;
		//------ ����ǰ��  --------------------------------------------------
		exp->EXP_Roll += FW*180;//240;//220;//180;//200;//240;
		//------ ��������  ---------------------------------------------------
		ZuoYouPID.Error = NavTempofC[0];
		ZuoYouPID.Error =30 -ZuoYouPID.Error;
		
// 		if(ZuoYouPID.Error>0 && ZuoYouPID.PIDInter_Sum<0) ZuoYouPID.PIDInter_Sum=0;
// 		else if(ZuoYouPID.Error<0 && ZuoYouPID.PIDInter_Sum>0) ZuoYouPID.PIDInter_Sum=0;
		
// 		if(ZuoYouPID.Error<20&&ZuoYouPID.Error>-20)  ZuoYouPID.Error=0;
		PID_Count(&ZuoYouPID);
		// ִ�о���
		exp->EXP_Pitch -= ZuoYouPID.PIDFin_Sum;
	}
}

/*****************************@��Ȩ luomin*********************************************************/
