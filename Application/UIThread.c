#include "RefereeThread.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"

#include "RefereeBehaviour.h"
#include "Client_UI.h"
#include "CanPacket.h"
#include "CalculateThread.h"
#include "UIThread.h"

//需要添加UI慢速
//需要添加底盘停止的UI 
//Graph_Data deng1,kuang,deng2,deng3,deng4,fu,imagex,imagex1,imagex2,imagex3,imagey,imagey1,imagey2,imagey3,imagey4;
//String_Data bullet,bullet3,DAFU,Abuff,Pbuff,Cbuff,state,ZIMIAO,dafustate,zimiaostate,dafustate1,zimiaostate1,state4;
//Float_Data capacityD,Min,Sec;
//int Time=0,M=10,S=0;
//int BTime=0,BM=10,BS=0;
//Float_Data BMin,BSec;

//Graph_Data Frame;//自瞄框
////准星横线
//Graph_Data Collimation_1;
//Graph_Data Collimation_2;
//Graph_Data Collimation_3;
//Graph_Data Collimation_4;
////准星数线
//Graph_Data Collimation_5;
////准心距离线
//Graph_Data Collimation_6;
//Graph_Data Collimation_7;
//Graph_Data Collimation_8;
//Graph_Data Collimation_9;
//Graph_Data Collimation_10;
//Graph_Data Collimation_11;

//String_Data Capcity,HitRune,AiMBot;
//Float_Data CapData;
//String_Data HitRuneStatus,AiMBotStatus;

//String_Data BulletCover,ChassisStatue;
//Graph_Data BulletCircle,ChassisStatueCircle;

//String_Data ChassisMove;
//Graph_Data ChassisMoveCircle;

//String_Data s_rune,b_rune;

//uint32_t flash_time = 0;
//uint8_t s_rune_flag = 0;
//uint8_t b_rune_flag = 0;
//int s_time_rune;
//int b_time_rune;

Graph_Data imagex,imagey,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14;
Graph_Data circle1,circle2,circle3,circle4;
Graph_Data Pingheng;
String_Data Ammo,Heat,aimbot,autofire,Mode,close1,open1,open2,close2,open3,close3,open4,close4,open5,close5,roting,noforce,fallow,stop;
String_Data capcity;
Float_Data CapData;
int mode_flag=0;

extern float CapChageVoltage;
extern EulerSystemMeasure_t    Imu;


void UI(void const * argument)
{
	
	
	 while(1)
    {
		//固定UI图层
			Line_Draw(&imagex,"xck",UI_Graph_ADD,0,UI_Color_Green,2,6900,540,7310,540);
		  Line_Draw(&imagey,"yck",UI_Graph_ADD,0,UI_Color_Green,1,7105,900,7105,100);
			UI_ReFresh(2,imagex,imagey);
			Line_Draw(&x1,"x01",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,520,7160,520);
//			Line_Draw(&x2,"x02",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,500,7160,500);
//			Line_Draw(&x3,"x03",UI_Graph_ADD,0,UI_Color_Cyan,1,7070,480,7140,480);
//			Line_Draw(&x4,"x04",UI_Graph_ADD,0,UI_Color_Cyan,1,7070,460,7140,460);
//			Line_Draw(&x5,"x05",UI_Graph_ADD,0,UI_Color_Cyan,1,7085,440,7125,440);
//				UI_ReFresh(5,x1,x2,x3,x4,x5);
			UI_ReFresh(1,x1);
			Line_Draw(&x6,"x06",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,420,7160,420);
//			Line_Draw(&x7,"x07",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,400,7160,400);
//			Line_Draw(&x8,"x08",UI_Graph_ADD,0,UI_Color_Cyan,1,7070,380,7140,380);
//			Line_Draw(&x9,"x09",UI_Graph_ADD,0,UI_Color_Cyan,1,7070,360,7140,360);
//			Line_Draw(&x10,"x10",UI_Graph_ADD,0,UI_Color_Cyan,1,7085,340,7125,340);
//				UI_ReFresh(5,x6,x7,x8,x9,x10);
			UI_ReFresh(1,x6);
//			Line_Draw(&x11,"x11",UI_Graph_ADD,1,UI_Color_Pink,6,6790,140,7420,140);
//			Line_Draw(&x12,"x12",UI_Graph_ADD,1,UI_Color_Pink,6,6900,320,7310,320);
			Line_Draw(&x13,"x13",UI_Graph_ADD,1,UI_Color_Pink,6,6706,40,6910,340);
			Line_Draw(&x14,"x14",UI_Graph_ADD,1,UI_Color_Pink,6,7504,40,7300,340);
			
//				UI_ReFresh(2,x11,x12);
				UI_ReFresh(2,x13,x14);	
			UI_ReFresh(1,x11);
			Line_Draw(&Pingheng,"pingheng",UI_Graph_ADD,0,UI_Color_Cyan,8,800,100,1000,100);
				UI_ReFresh(1,Pingheng);
			Char_Draw(&Mode,"mod",UI_Graph_ADD,0,UI_Color_Green,14,5,2,6750,770,"MODE");
			Char_ReFresh(Mode);
			Char_Draw(&autofire,"fir",UI_Graph_ADD,0,UI_Color_Green,14,5,2,6750,740,"FIRE");
			Char_ReFresh(autofire);
			Char_Draw(&aimbot,"aim",UI_Graph_ADD,0,UI_Color_Green,14,5,2,6750,710,"SINGLE");
			Char_ReFresh(aimbot);
			Char_Draw(&Heat,"hea",UI_Graph_ADD,0,UI_Color_Green,14,5,2,6750,800,"cover");
			Char_ReFresh(Heat);

			Char_Draw(&open1,"op",UI_Graph_ADD,5,UI_Color_Purplish_red,16,7,2,6820,800,"open");
				Char_ReFresh(open1);
				
				Char_Draw(&open2,"opp",UI_Graph_ADD,5,UI_Color_Purplish_red,16,7,2,6820,740,"close ");
				Char_ReFresh(open2);
			Char_Draw(&noforce,"nof",UI_Graph_ADD,5,UI_Color_Purplish_red,16,7,2,6820,770,"noforce");
				Char_ReFresh(noforce);
				Char_Draw(&open3,"oop",UI_Graph_ADD,5,UI_Color_Purplish_red,16,7,2,6820,710,"close ");
				Char_ReFresh(open3);
				
			//************************************弹舱盖**********************************	
			if( (PTZ.PTZStatusInformation   & 16 ) == 16)
			{
				Char_Draw(&open1,"op",UI_Graph_Change,5,UI_Color_Purplish_red,16,7,2,6820,800,"open  ");
				Char_ReFresh(open1);
			}
			else
			{
				Char_Draw(&open1,"op",UI_Graph_Change,5,UI_Color_Purplish_red,16,7,2,6820,800,"close ");
				Char_ReFresh(open1);
				
			}
			//************************************自动开火********************************
			if(  (PTZ.PTZStatusInformation     &  64) == 64)
			{
				Char_Draw(&open2,"opp",UI_Graph_Change,5,UI_Color_Purplish_red,16,7,2,6820,740,"open  ");
				Char_ReFresh(open2);
			}
			else
			{
				Char_Draw(&open2,"opp",UI_Graph_Change,5,UI_Color_Purplish_red,16,7,2,6820,740,"close ");
				Char_ReFresh(open2);
			
			}	
			//*********************************单发*********************************
				if(   (PTZ.AimTargetRequest & 0x02) == 0x02){
					Char_Draw(&open3,"oop",UI_Graph_Change,5,UI_Color_Purplish_red,16,7,2,6820,710,"open  ");
					Char_ReFresh(open3);
			}
			else
			{
					Char_Draw(&open3,"oop",UI_Graph_Change,5,UI_Color_Purplish_red,16,7,2,6820,710,"close ");
					Char_ReFresh(open3);
				
				}
				
				
				if(Chassis.Mode == NOFORCE)
				{

					Char_Draw(&noforce,"nof",UI_Graph_Change,5,UI_Color_Purplish_red,16,7,2,6820,770,"noforce");
					Char_ReFresh(noforce);
				}
				else if(Chassis.Mode == ROTING)
				{

					Char_Draw(&noforce,"nof",UI_Graph_Change,5,UI_Color_Green,16,7,2,6820,770,"rotate");
					Char_ReFresh(noforce);
				}
				else if(Chassis.Mode == FALLOW)
				{
				 
					Char_Draw(&noforce,"nof",UI_Graph_Change,5,UI_Color_Cyan,16,7,2,6820,770,"fallow");
					Char_ReFresh(noforce);
				}
				else if(Chassis.Mode == STOP)
				{
					Char_Draw(&noforce,"nof",UI_Graph_Change,5,UI_Color_Cyan,16,7,2,6820,770,"stop");
					Char_ReFresh(noforce);
				}
				else if(Chassis.Mode == HIGHSPEED)
				{
					Char_Draw(&noforce,"nof",UI_Graph_Change,5,UI_Color_Cyan,16,7,2,6820,770,"Fly");
					Char_ReFresh(noforce);
				
				}

			
			

			//底盘平衡指示
				Line_Draw(&Pingheng,"pingheng",UI_Graph_Change,0,UI_Color_Cyan,8,800,100-5*Imu.PitchAngle,1000,100+5*Imu.PitchAngle);
				UI_ReFresh(1,Pingheng);
			
			
        osDelay(10);
    }

 }