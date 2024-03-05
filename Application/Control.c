#include "Control.h"
#include "arm_math.h"

#define printf(...)  HAL_UART_Transmit_DMA(&huart1,\
																				(uint8_t  *)u1_buf,\
																				sprintf((char*)u1_buf,__VA_ARGS__))
uint8_t u1_buf[28];

float LEG_KP	=																			800.0f;
float LEG_KI	=																			0.5f;
float LEG_KD	=																			8000.0f;

float LEG_KP_DOWN	=																			1000.0f;
float LEG_KI_DOWN	=																			0.0f;
float LEG_KD_DOWN	=																			10000.0f;
float Control_Kp_2x[6][2][4]={
-147.572221,166.673514,-99.732531,-0.762048,
228.695589,-192.918548,49.139160,2.784781,
14.101269,-13.243259,-6.325914,-0.194540,
18.906468,-19.271447,7.677948,0.249342,
-52.624260,53.886109,-20.321851,-1.917153,
-13.630290,30.522401,-21.940247,7.013751,
-35.848298,40.695509,-19.112699,-2.349499,
-14.533191,29.311559,-20.636601,7.037613,
19.709409,22.727210,-32.931686,11.782952,
163.464004,-173.557404,67.504466,10.660965,
-15.532233,18.912462,-9.679073,2.395525,
37.560055,-36.951604,13.186448,0.328670
};

float Control_Kp_x[6][2][4]={
-138.725513,140.740893,-76.884097,-2.946844,
439.989524,-331.324686,82.760427,0.416829,
46.096813,-34.076302,1.606678,-1.064437,
52.462321,-38.648237,10.486367,0.111468,
14.207189,-5.388246,-1.411512,-2.693371,
70.861780,-41.347904,2.316052,3.138494,
51.523359,-32.494809,4.045611,-3.953803,
79.209017,-44.825993,1.700296,3.948467,
102.791549,-44.145953,-12.420259,11.139399,
-58.867023,3.429718,20.716371,16.547152,
-29.439705,25.034911,-9.107553,2.236749,
9.509343,-12.309505,6.118646,0.933139
};

float Control_Kp[6][2][4]={
-95.744604,128.525068,-92.763243,-0.274430,
481.090561,-419.688451,113.773694,3.859422,
20.473840,-18.460926,-5.122783,-0.159249,
36.752891,-39.231243,16.022912,0.315007,
-53.130154,59.739851,-24.865454,-0.858261,
34.737307,3.500403,-25.322610,11.741300,
-35.412977,45.616447,-23.387305,-1.290867,
32.061865,1.673127,-22.880552,11.806338,
87.944656,-29.416492,-23.623901,13.234245,
262.444981,-313.356276,136.241453,11.007915,
-7.744292,12.154344,-7.857104,2.313955,
60.986917,-63.118414,23.947251,-0.781886
};
float Control_K_extern[10][4][4]={
-42.140113,55.377265,-55.606176,0.071554,
271.407081,-250.230288,104.304803,1.376512,
-87.198960,102.817062,-46.081509,-0.182965,
167.830095,-117.844032,-20.776164,3.520969,
12.562441,-13.407670,-1.916951,-0.173761,
9.888561,-9.918635,8.310140,0.440580,
4.737149,-2.115176,-4.031734,0.021359,
27.662807,-28.934127,4.204676,0.040417,
-87.198960,102.817062,-46.081509,-0.182965,
167.830095,-117.844032,-20.776164,3.520969,
-42.140113,55.377265,-55.606176,0.071554,
271.407081,-250.230288,104.304803,1.376512,
4.737149,-2.115176,-4.031734,0.021359,
27.662807,-28.934127,4.204676,0.040417,
12.562441,-13.407670,-1.916951,-0.173761,
9.888561,-9.918635,8.310140,0.440580,
-46.092107,51.403843,-23.615534,2.839605,
21.196907,-5.444622,-6.397996,9.527683,
-63.781658,61.592021,-18.368030,-4.034237,
2.116118,38.457387,-40.024323,5.907725,
-24.361057,30.019042,-16.631962,1.114214,
20.598081,-7.899676,-4.141824,7.036211,
-31.088584,33.002668,-12.048145,-2.655071,
-12.472407,35.366197,-29.735606,4.759296,
-63.781658,61.592021,-18.368030,-4.034237,
2.116118,38.457387,-40.024323,5.907725,
-46.092107,51.403843,-23.615534,2.839605,
21.196907,-5.444622,-6.397996,9.527683,
-31.088584,33.002668,-12.048145,-2.655071,
-12.472407,35.366197,-29.735606,4.759296,
-24.361057,30.019042,-16.631962,1.114214,
20.598081,-7.899676,-4.141824,7.036211,
63.483953,2.888572,-39.338561,13.711530,
366.055037,-386.620087,146.073839,9.959267,
63.483953,2.888572,-39.338561,13.711530,
366.055037,-386.620087,146.073839,9.959267,
-10.707518,16.415856,-10.174090,2.391287,
70.564208,-69.466167,24.233822,-0.477125,
-10.707518,16.415856,-10.174090,2.391287,
70.564208,-69.466167,24.233822,-0.477125
};
/*

-187.232444,176.571820,-73.626573,-3.405606,
372.181900,-264.717279,51.643899,6.926407,
32.514508,-20.937075,1.117895,-1.280556,
34.801307,-21.422668,2.297874,1.772903,
1.597747,-0.381800,-0.345536,-0.365727,
10.979647,-6.194226,0.050697,0.634606,
16.432462,-9.663349,0.957954,-1.709930,
30.114866,-13.251718,-3.130082,2.865065,
94.726591,-34.410009,-18.631277,13.120414,
-16.231501,-51.759515,47.175949,20.705837,
-23.101850,20.194273,-7.747819,1.969718,
9.571109,-11.986133,5.910807,1.398561

-187.232444,176.571820,-73.626573,-3.405606,
372.181900,-264.717279,51.643899,6.926407,
32.514508,-20.937075,1.117895,-1.280556,
34.801307,-21.422668,2.297874,1.772903,
1.597747,-0.381800,-0.345536,-0.365727,
10.979647,-6.194226,0.050697,0.634606,
16.432462,-9.663349,0.957954,-1.709930,
30.114866,-13.251718,-3.130082,2.865065,
94.726591,-34.410009,-18.631277,13.120414,
-16.231501,-51.759515,47.175949,20.705837,
-23.101850,20.194273,-7.747819,1.969718,
9.571109,-11.986133,5.910807,1.398561

-53.271572,80.210276,-60.475469,-3.560139,
675.783066,-503.068428,125.038383,-1.636299,
61.991203,-44.655022,4.544718,-1.139904,
83.205852,-61.263669,16.618960,-0.306592,
38.198679,-21.502462,1.813631,-3.194091,
177.970318,-113.341816,15.985481,3.674588,
81.053000,-53.413889,8.889092,-3.987632,
170.317703,-107.534594,15.027946,3.790918,
282.831695,-169.127926,14.624296,10.390498,
-287.678572,149.188159,-3.699706,25.079257,
-21.213283,19.331398,-7.884255,2.197823,
-4.961014,-4.024658,5.325812,1.108339
*/

fp32 a_right,da_right,x_right,dx_right,b_right,db_right,last_a_right,a_left,da_left,x_left,dx_left,b_left,db_left,last_a_left;
fp32 T_left1,T_left2,T_right1,T_right2,T_right,T_left,F_right,F_left,T_left1_r,T_left2_r,T_right1_r,T_right2_r;
fp32 u1_right,u4_right,du1_right,du4_right;
fp32 d2u1_right,d2u4_right,d2u1_left,d2u4_left;
fp32 u1_left,u4_left,du1_left,du4_left,tt,sinu2u3;
fp32 k1,k2,k3,cosu1,cosu4,sinu1,sinu4;

fp32 u0,u2,u3;
fp32 xd,yd,xb,yb,lbd,A10,B10,C10,xc,yc,l0;
fp32 dxc,dyc,d2xc_left,d2xc_right;
fp32 T,Tp_left,Tp_right,N,P,Nm,Pm,R,g,L,Lm,l,mw,mp,M,Iw,Ip,Im;	
// -2500 -600 -0.08 -0.l
fp32 l1,l2,W,u1,u4,du1,du4;
fp32 lt,xt_right,xt_left,lt_left,lt_right,dxt_left,dxt_right;
fp32 Kp_th=-80000,Kp_dth=-8000,Kp_x=-0.075,Kp_v=-0.05,Kp_ya=-9,Kp_ys=-4.5,kp_stop=5000,k0=0,d0=0,kl=0.0;

uint8_t last_state=0;
	 
first_order_filter_type_t angle_filter,speed_filter,lright_filter,lleft_filter,dlleft_filter,Fleft_filter,dlright_filter,Fright_filter;
first_order_filter_type_t aleft_filter,aright_filter,daleft_filter,daright_filter,x_filter,dx_filter,xt_filter;
first_order_filter_type_t u1_right_filter,u4_right_filter,u1_left_filter,u4_left_filter;
first_order_filter_type_t du1_right_filter,du4_right_filter,du1_left_filter,du4_left_filter;
first_order_filter_type_t xleft_filter,xright_filter,dxleft_filter,dxright_filter;
fp32 paramater0[1]={0.00};	
fp32 paramater1[1]={0.002};	 
fp32 paramater2[1]={0.00};	
fp32 paramater3[1]={0.06};	 
fp32 paramater4[1]={0.01};	 
fp32 paramater5[1]={0.1};
fp32 paramater6[1]={0.0};	
first_order_filter_type_t vxt_filter;
uint8_t jump_state=0,chassis_state=0;

pid_type_def follow_pid;
pid_type_def turn_pid;
pid_type_def roll_pid;
pid_type_def leg_pid_left,leg_pid_right;

fp32 follow_PID[3]={FOLLOW_KP,FOLLOW_KI,FOLLOW_KD};
fp32 turn_PID[3]={TURN_KP,TURN_KI,TURN_KD};
fp32 roll_PID[3]={ROLL_KP,ROLL_KI,ROLL_KD};
//fp32 leg_PID[3]={LEG_KP,LEG_KI,LEG_KD};

fp32 Pout,Dout;

int cal_flag=0;

float Calc_Poly_extern(float x,int j,int i)
{
		return x*x*x*Control_K_extern[i][j][0]+x*x*Control_K_extern[i][j][1]+x*Control_K_extern[i][j][2]+Control_K_extern[i][j][3];
}
float Calc_Poly(float x,int j,int i)
{
		return x*x*x*Control_Kp[i][j][0]+x*x*Control_Kp[i][j][1]+x*Control_Kp[i][j][2]+Control_Kp[i][j][3];
}
float Calc_Poly_a(float x,int j,int i)
{
		return x*x*x*Control_Kp_x[i][j][0]+x*x*Control_Kp_x[i][j][1]+x*Control_Kp_x[i][j][2]+Control_Kp_x[i][j][3];
}
float Sqrt(float x)
{
		float x_in,y_out;
		x_in=x;
		arm_sqrt_f32(x_in,&y_out);
		return y_out;
}
void Calculate_Init()
{
    //crc32_init(0x04c11db7);
		a_right=0;da_right=0;
		b_right=0;db_right=0;
		x_right=0;dx_right=0;
		last_a_right=0;
		a_left=0;da_left=0;
		b_left=0;db_left=0;
		x_left=0;dx_left=0;
		last_a_left=0;
	
		R=0.1f;
		l1=0.12f;
		l2=0.24f;
		W=0.15f;
	
		xt_right=0;
		xt_left=0;
	
		mw=1.14f;
	
		first_order_filter_init(&u1_right_filter,0.001,paramater0);
		first_order_filter_init(&u4_right_filter,0.001,paramater0);
		first_order_filter_init(&u1_left_filter,0.001,paramater0);
		first_order_filter_init(&u4_left_filter,0.001,paramater0);
		first_order_filter_init(&du1_right_filter,0.001,paramater1);
		first_order_filter_init(&du4_right_filter,0.001,paramater1);
		first_order_filter_init(&du1_left_filter,0.001,paramater1); 
		first_order_filter_init(&du4_left_filter,0.001,paramater1);

		first_order_filter_init(&angle_filter, 0.001, paramater0);
		first_order_filter_init(&speed_filter,0.001,paramater6);
		first_order_filter_init(&x_filter, 0.001, 0);
		first_order_filter_init(&dx_filter,0.001,0);
		first_order_filter_init(&xleft_filter, 0.001, paramater4);
		first_order_filter_init(&dxleft_filter,0.001,paramater3);
		first_order_filter_init(&xright_filter, 0.001, paramater4);
		first_order_filter_init(&dxright_filter,0.001,paramater3);		
		first_order_filter_init(&aleft_filter,0.001,paramater2);
		first_order_filter_init(&daleft_filter,0.001,paramater2);
		first_order_filter_init(&aright_filter,0.001,paramater2);
		first_order_filter_init(&daright_filter,0.001,paramater2);
		first_order_filter_init(&lleft_filter,0.001,paramater2);
		first_order_filter_init(&lright_filter,0.001,paramater2);
		first_order_filter_init(&dlleft_filter,0.001,paramater2);
		first_order_filter_init(&dlright_filter,0.001,paramater2);
		first_order_filter_init(&Fleft_filter,0.001,paramater2);
		first_order_filter_init(&Fright_filter,0.001,paramater2);
		first_order_filter_init(&vxt_filter,0.001,paramater5);
		first_order_filter_init(&xt_filter,0.001,paramater5);
		
		PID_init(&follow_pid,PID_POSITION,follow_PID,10,5);
		PID_init(&turn_pid,PID_POSITION,turn_PID,30000,5000);
		PID_init(&roll_pid,PID_POSITION,roll_PID,1,0);
		//PID_init(&leg_pid_left,PID_POSITION,leg_PID,320,60);
		//PID_init(&leg_pid_right,PID_POSITION,leg_PID,320,60);
		
}
float KLEFT1=-3.36f,KLEFT2=3.86f,KRIGHT1=-4.15f,KRIGHT2=3.53f;

float Calc_pos(float pos,ID id)
{
	if(id==LEFT1)
		return 3.14159f-pos/9.1f+KLEFT1;
	if(id==LEFT2)
		return -pos/9.1f+KLEFT2;
	if(id==RIGHT1)
		return 3.14159f+pos/9.1f+KRIGHT1;
	if(id==RIGHT2)
		return pos/9.1f+KRIGHT2;
	return 0;
}
void Change(float angle,ID id)
{
		if(id==LEFT1)
				KLEFT1+=-0.52359f-angle;
		if(id==RIGHT1)
				KRIGHT1+=-0.52359f-angle;
		if(id==LEFT2)
				KLEFT2+=3.66519f-angle;
		if(id==RIGHT2)
				KRIGHT2+=3.66519f-angle;
}

fp32 ps,pa,tht;

float dl0_right,dl0_left,l_dl0_left,l_dl0_right,count1=0,count2=0,I_right,I_left,l_l0_right,l_l0_left,i_l0_right,i_l0_left,FALLOW_Angle;
float l_u3_left,l_u3_right;
float	follow_angle,wz_current;	
float FALLOW_angle=FALLOW_ANGLE,debug=2.0f;
float Leg_Force;
int ct=0,init_time;

float lt0=0.2f,i_rollangle=0;
float Time=0,ticking=0;
int mo=0;
int stand_flag=-1;
int change_flag=0,tic=1;
float last_dx_left,last_dx_right;
void ChassisStateUpdate()
{
		Imu.YawAngle=INS_angle[0]/3.14159f*180.0f;	
		Imu.PitchAngle=90.0f-INS_angle[2]/3.14159f*180.0f+debug;
		Imu.RollAngle=INS_angle[1]/3.14159f*180.0f-3.8f;
		Imu.YawSpeed=-GyroCorrected[2]/3.14159f*180.0f;	
		Imu.PitchSpeed=GyroCorrected[1]/3.14159f*180.0f;
		Imu.RollSpeed=-GyroCorrected[0]/3.14159f*180.0f;		
		
	
		first_order_filter_cali(&angle_filter,Imu.PitchAngle);
		first_order_filter_cali(&speed_filter,Imu.PitchSpeed);	
	
	
		if(change_flag)
		{
				Change(u1_right,RIGHT2);
				Change(u4_right,RIGHT1);
				Change(u1_left,LEFT2);
				Change(u4_left,LEFT1);
		}
	
	
		u4_left=Calc_pos(motor_left1_r.Pos,LEFT1);
		du4_left=-motor_left1_r.W/9.1f;
		d2u4_left=motor_left1_r.Acc;	
		T_left1_r=motor_left1_r.T;
		u1_left=Calc_pos(motor_left2_r.Pos,LEFT2);
		du1_left=-motor_left2_r.W/9.1f;
		d2u1_left=motor_left2_r.Acc;	
		T_left2_r=motor_left2_r.T;	
		u4_right=Calc_pos(motor_right1_r.Pos,RIGHT1);
		du4_right=motor_right1_r.W/9.1f;		
		d2u4_right=motor_right1_r.Acc;	
		T_right1_r=motor_right1_r.T;	
		u1_right=Calc_pos(motor_right2_r.Pos,RIGHT2);
		du1_right=motor_right2_r.W/9.1f;	
		d2u1_right=motor_right2_r.Acc;	
		T_right2_r=motor_right2_r.T;			
	
	
	
		
		
			
		
		float x,xt,v;
		b_right=angle_filter.out*3.14159f/180.0f;
		db_right=speed_filter.out*3.14159f/180.0f;
		b_left=angle_filter.out*3.14159f/180.0f;
		db_left=speed_filter.out*3.14159f/180.0f;		
		x=(x_left+x_right)/2.0f;
		v=(dx_left+dx_right)/2.0f;
		first_order_filter_cali(&x_filter,x);
		first_order_filter_cali(&dx_filter,v);	
	
		if(tic==1)
			printf("%f,%f,%d;\n",last_dx_left,xleft_filter.out,LeftFootMotorMeasure.given_current);
		last_dx_left=xleft_filter.out;
		last_dx_right=xright_filter.out;		
		tic=(tic+1)%4;
}

void ChassisModeUpdate()
{
		count_time[3]++;
		/*
		switch(PTZ.ChassisStatueRequest)
		{
				case 0x01:
						Chassis.Mode = NOFORCE; break;
				case 0x12:
				case 0x92:
						Chassis.Mode = ROTING; break;
				case 0x0A:
				case 0x8A:
						Chassis.Mode = FALLOW; break;
				case 0x06:
				case 0x86:
						Chassis.Mode = STOP; break;
				case 0x2A:
				case 0xAA:
						Chassis.Mode = HIGHSPEED; break;
				case 0x42:
				case 0xC2:
						Chassis.Mode = JUMP; break;
				default:
						break;
		}	*/
		if(rc_ctrl.rc.s[0]==1)
				if(rc_ctrl.rc.ch[4]!=660)
						Chassis.Mode = FALLOW;
				else
						Chassis.Mode = JUMP;
		else
				Chassis.Mode = NOFORCE;
//		if(PTZ.ChassisStatueRequest&(1<<7))
//				stand_flag=-1;
//		else
				stand_flag=-1;
		if(rc_ctrl.rc.s[1]==1)
				ticking+=0.0314*3;
		else
				ticking=0;
		if(Chassis.Mode==NOFORCE||total_time[0]<=500||total_time[1]<=500)
		{		
				joint_motor_set_mode(mo);  
				chassis_state=0;
				//x_left=0;
				//x_right=0;
				//xt_left=0;
				//xt_right=0;
		}
		else
		{
				joint_motor_set_mode(10);  
				if(last_state==NOFORCE||last_state==HIGHSPEED&&Chassis.Mode!=HIGHSPEED)
				{
						chassis_state=1;
						init_time=0;
				}
				if(chassis_state==1&&init_time>500)
				{
						chassis_state=2;
						x_left=0;
						x_right=0;
						xt_left=0;
						xt_right=0;
						init_time=0;
				}
				if(chassis_state==2&&init_time>800&&total_time[2]>=3900)
				{
						chassis_state=3;
						x_left=0;
						x_right=0;
						xt_left=0;
						xt_right=0;
						Change(u1_right,RIGHT2);
						Change(u4_right,RIGHT1);
						Change(u1_left,LEFT2);
						Change(u4_left,LEFT1);
						xt_filter.out=0;
						i_rollangle=0;
						count1=0;
						count2=0;
				}
				if(chassis_state==3)
				{
						if(Imu.PitchAngle>29.5f||Imu.PitchAngle<-29.5f)
						{
								chassis_state=1;
								init_time=0;
						}
				}
				if(Chassis.Mode==HIGHSPEED)
				{
						chassis_state=4;
				}	
				if(Chassis.Mode==JUMP&&last_state!=JUMP)
				{
						chassis_state=5;
						init_time=0;
				}	
				if(Chassis.Mode!=JUMP&&chassis_state==5)
				{
						chassis_state=6;
						init_time=0;
						//x_left=0;
						//x_right=0;
						//xt_left=0;
						//xt_right=0;		
				}
				if(chassis_state==6&&((l0_left>0.3&&l0_right>0.3)||init_time>300))
				{
						chassis_state=7;
						init_time=0;
				}
				if(chassis_state==7&&(init_time>500))
				{
						chassis_state=3;
						init_time=0;
				}
//				if(chassis_state==7&&init_time>300)
//				{
//						chassis_state=3;
//						init_time=0;
//				}

//				if((Chassis.Mode==FALLOW||Chassis.Mode==ROTING||Chassis.Mode==STOP)&&chassis_state>=3)
//				{
//						chassis_state=3;
//				}
		}
		
		last_state=Chassis.Mode;	
		
		//Chassis.vx=rc_ctrl.rc.ch[1]*32767/660;
		if(Chassis.Mode!=STOP)
		{
				first_order_filter_cali(&xt_filter,xt_filter.out-rc_ctrl.rc.ch[1]/660.0f*2.1f);	
				first_order_filter_cali(&vxt_filter,-rc_ctrl.rc.ch[1]/660.0f*2.1f);	
		}
		else
		{
				first_order_filter_cali(&xt_filter,xt_filter.out+rc_ctrl.rc.ch[1]/660.0f*2.1f);	
				first_order_filter_cali(&vxt_filter,rc_ctrl.rc.ch[1]/660.0f*2.1f);	
		}
		Chassis.vx=-rc_ctrl.rc.ch[1]/660.0f*2.0f;
		Chassis.wz=-rc_ctrl.rc.ch[2]/660.0f*10.0f;
		//xt_left=xt_filter.out
		//xt=(xt_left+xt_right)/2.0f-PTZ.FBSpeed/32767.0f*2.15f*0.00125f;
		//Chassis.vx=TD_calc(&remote_td,xt_filter.out);
		

		//x_left=x;x_right=x;
		//dx_left=v;dx_right=v;			
		//Wheel_Update(v);
		
}
float temp_p,avr=0,cop=0,ktf=0.75f;
uint8_t cflag=0;
bool stop_flag=0;
float fsgn(float x)
{
		if(x>0)	return 1;
		if(x<0)	return -1;
		return 0;
}

void Calculate_LQR()
{
		correct_time++;
		float x,xt,v;
		x=(x_left+x_right)/2.0f;
		v=(dx_left+dx_right)/2.0f;	

		avr=v;
		xt=(xt_left+xt_right)/2.0f;

		
		if(chassis_state==0)
		{  
				Calculate_right();
				Calculate_left();
				wz_current=0;	
				I_right=0;
				I_left=0;
				T_left1=0;
				T_left2=0;
				T_right1=0;
				T_right2=0;			
				i_l0_right=0;
				i_l0_left=0;			
		}
		if(chassis_state==1)
		{
				T_left1=0.3;
				T_left2=-0.3;
				T_right1=-0.3;
				T_right2=0.3;	
				I_right=-kp_stop*(dx_right);
				I_left=kp_stop*(dx_left);		
				init_time++;			
		}
		if(chassis_state==2)
		{
	
	
//			
				tht=(x+xt)*Kp_x+(v+Chassis.vx)*Kp_v;
//							
//				follow_angle=loop_fp32_constrain(FALLOW_angle, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
//				Chassis.wz=PID_calc(&follow_pid,YawMotorMeasure.angle,follow_angle)*3.14159f/180.0f;
//				wz_current=PID_calc(&turn_pid,-(RightFootMotorMeasure.speed_rpm+LeftFootMotorMeasure.speed_rpm)*0.00136f,-Chassis.wz);	
//								
			
				wz_current=0;
				I_right=Kp_th*(b_right+tht)+Kp_dth*db_right+wz_current;
				I_left=-Kp_th*(b_right+tht)-Kp_dth*db_right+wz_current;
				T_left1=0.3;
				T_left2=-0.3;
				T_right1=-0.3;
				T_right2=0.3;									
				init_time++;
				FALLOW_Angle=Imu.YawAngle;
		}
		
		if(chassis_state==3||chassis_state>4)
		{
				if(chassis_state>4)
						init_time++;
				if(chassis_state==3||chassis_state==7)
						lt=lt0+arm_sin_f32(ticking)*0.045;//+rc_ctrl.rc.ch[4]*0.1f/660.0f;//+0.045f+stand_flag*0.045f;-fabs(Imu.PitchAngle)/600.0f;
				if(chassis_state==5)
						lt=0.1f;
				if(chassis_state==7)
						lt=0.12f;				
//				if(Chassis.Mode==ROTING)
//				{	
//						Chassis.wz=ROTING_SPEED;					
//				}
//				if(Chassis.Mode==FALLOW||Chassis.Mode==JUMP)
//				{
//						follow_angle=loop_fp32_constrain(FALLOW_Angle, Imu.YawAngle - 180.0f,Imu.YawAngle + 180.0f);
//						Chassis.wz=-PID_calc(&follow_pid,Imu.YawAngle,follow_angle);					
//				}
//				if(Chassis.Mode==STOP)
//				{
//						follow_angle=loop_fp32_constrain(FALLOW_angle+90.0f, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
//						Chassis.wz=PID_calc(&follow_pid,YawMotorMeasure.angle,follow_angle)*3.14159f/180.0f;				
//				}
				//wz_current=PID_calc(&turn_pid,-(RightFootMotorMeasure.speed_rpm+LeftFootMotorMeasure.speed_rpm)*0.00136f,-Chassis.wz);	
				
				float len=(-Imu.RollAngle*3.14/180)*ROLL_KP-i_rollangle*ROLL_KI+Imu.RollSpeed*3.14/180*ROLL_KD;
				if(count1<0&&count2<0)
						i_rollangle+=Imu.RollAngle*3.14/180;
				Leg_Force=len;
				Pout=Imu.RollAngle*ROLL_KP; 
				Dout=Imu.RollSpeed*ROLL_KD;
				if(fabs(len)>lt-0.11f)	
				{
						if(fabs(len)+0.11f>0.31f)	len=0.2f;
						lt_left=fabs(len)+0.11f+len;
						lt_right=fabs(len)+0.11f-len;							
				}
				else
				{
						lt_left=lt+len;
						lt_right=lt-len;		
				}
//				if(Chassis.Mode!=STOP)
//						temp_p=xt_filter.out+x+Chassis.vx*0.15f; 
//				else
//						temp_p=xt_filter.out+x+Chassis.vx*0.3f; 
//				if(temp_p>fabs(Chassis.vx)+0.5f)	temp_p=fabs(Chassis.vx)+0.5f;
//				if(temp_p<-fabs(Chassis.vx)-0.5f)	temp_p=-fabs(Chassis.vx)-0.5f;			
				
				//xt_filter.out=temp_p-x;

				

				
//				if(Chassis.vx==0)
//				{
//						if(stop_flag==1)
//								xt_filter.out=xt_filter.out*0.9+x*0.1,
//								stop_flag=0;
//				}
//				else
//						stop_flag=1;
				//_filter.out+=x;
				//if(xt_filter.out>1.0f)	xt_filter.out=1.0f;
				//if(xt_filter.out<-1.0f)	xt_filter.out=-1.0f;
				//x_left=0;
				//x_right=0;
//				xt=(xt_left+xt_right)/2.0f+Chassis.vx*0.00125f;
				
//				if(xt+x+v+Chassis.vx*0.5<-2.0f)
//						xt=-2.0f-x-v-Chassis.vx*0.5;
//				
//				if(xt+x+v+Chassis.vx*0.5>2.0f)
//						xt=2.0f-x-v-Chassis.vx*0.5;
				//xt_left=remote_td.x1;
				//xt_right=remote_td.x1;		
//				xt_left+=Chassis.vx*0.00125f+Chassis.wz*0.001f;
//				xt_right+=Chassis.vx*0.00125f-Chassis.wz*0.001f;

				

			

				
				
//				if(T_right*TORQUE_K+wz_current>16383)
//						wz_current=16383-T_right*TORQUE_K;
//				if(T_right*TORQUE_K+wz_current<-16383)
//						wz_current=-16383-T_right*TORQUE_K;
//				if(T_left*TORQUE_K+wz_current>16383)
//						wz_current=16383-T_left*TORQUE_K;
//				if(T_left*TORQUE_K+wz_current<-16383)
//						wz_current=-16383-T_left*TORQUE_K;					
				Calculate_right();
				Calculate_left();
				Calculate_Kp();
				if(count1>2&&Chassis.Mode!=JUMP)
						I_right=0;
				else
						I_right=(T_right/*+dx_right/TORQUE_W*/)*TORQUE_K+wz_current;
				if(count2>2&&Chassis.Mode!=JUMP)
						I_left=0;
				else
						I_left=-(T_left/*+dx_left/TORQUE_W*/)*TORQUE_K+wz_current;	
						
		}
		if(chassis_state==4)
		{
				T_left1=0.3;
				T_left2=-0.3;
				T_right1=-0.3;
				T_right2=0.3;	
//				follow_angle=loop_fp32_constrain(FALLOW_angle, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
//				Chassis.wz=PID_calc(&follow_pid,YawMotorMeasure.angle,follow_angle)*3.14159f/180.0f;
//				wz_current=PID_calc(&turn_pid,-(RightFootMotorMeasure.speed_rpm+LeftFootMotorMeasure.speed_rpm)*0.00136f,-Chassis.wz);	
				
				I_right=-kp_stop*(dx_right+Chassis.vx)+wz_current;
				I_left=kp_stop*(dx_left+Chassis.vx)+wz_current;		
		}
		
		if(I_right>16383)
				I_right=16383;
		if(I_right<-16383)
				I_right=-16383;
		if(I_left>16383)
				I_left=16383;
		if(I_left<-16383)
				I_left=-16383;		
		FEET_CONTROL((int)(-I_left),(int)(-I_right));
		//FEET_CONTROL(0,0);

}
float a,da,d2a,b,db,d2b,x,dx,d2x,Fn_left,Fn_right;	
float ri,le;
float l0_left,l0_right,ja_left,jb_left,jc_left,jd_left,ja_right,jb_right,jc_right,jd_right;

void Calculate_right(void)
{
		
		float ja,jb,jc,jd,k,th_sin,th_cos,F_r,Tp_r;

		first_order_filter_cali(&u1_right_filter,u1_right);	
		u1=u1_right_filter.out;
		first_order_filter_cali(&u4_right_filter,u4_right);	
		u4=u4_right_filter.out;	
		first_order_filter_cali(&du1_right_filter,du1_right);	
		du1=du1_right_filter.out;
		first_order_filter_cali(&du4_right_filter,du4_right);	
		du4=du4_right_filter.out;	
		cosu1=arm_cos_f32(u1),cosu4=arm_cos_f32(u4),sinu1=arm_sin_f32(u1),sinu4=arm_sin_f32(u4);

		xb=l1*cosu1-W/2.0f;
		yb=l1*sinu1;
		xd=W/2.0f+l1*cosu4;
		yd=l1*sinu4;
		lbd=Sqrt((xd-xb)*(xd-xb)+(yd-yb)*(yd-yb));
		A10=2*l2*(xd-xb);
		B10=2*l2*(yd-yb);
		C10=lbd*lbd;
		u2=2*atan2((B10+Sqrt(A10*A10+B10*B10-C10*C10)),(A10+C10));
		u3=atan2(yb-yd+l2*arm_sin_f32(u2),xb-xd+l2*arm_cos_f32(u2));
		sinu2u3=arm_sin_f32(u2-u3);
		xc=xb+l2*arm_cos_f32(u2);
		yc=yb+l2*arm_sin_f32(u2);
		u0=atan2(yc,xc);
		l0=Sqrt(xc*xc+yc*yc);
		dxc=l1*arm_sin_f32(u1-u2)*arm_sin_f32(u3)/sinu2u3*du1+l1*arm_sin_f32(u3-u4)*arm_sin_f32(u2)/sinu2u3*du4;
		dyc=l1*arm_sin_f32(u1-u2)*arm_cos_f32(u3)/sinu2u3*du1-l1*arm_sin_f32(u3-u4)*arm_cos_f32(u2)/sinu2u3*du4;
		d2xc_right=l1*arm_sin_f32(u1-u2)*arm_sin_f32(u3)/sinu2u3*d2u1_right+l1*arm_sin_f32(u3-u4)*arm_sin_f32(u2)/sinu2u3*d2u4_right;
		l_dl0_right=dl0_right;
		dl0_right=(xc*dxc+yc*dyc)/l0;
		first_order_filter_cali(&dlright_filter,dl0_right);
		dl0_right=dlright_filter.out;
		da_right=(dxc*yc-xc*dyc)/(xc*xc+yc*yc)-db_right;
		a_right=1.5708f-(b_right+u0);
		
		ja=l1*arm_sin_f32(u0-u3)*arm_sin_f32(u1-u2)/-sinu2u3;
		jb=l1*arm_cos_f32(u0-u3)*arm_sin_f32(u1-u2)/(l0*-sinu2u3);
		jc=l1*arm_sin_f32(u0-u2)*arm_sin_f32(u3-u4)/-sinu2u3;
		jd=l1*arm_cos_f32(u0-u2)*arm_sin_f32(u3-u4)/(l0*-sinu2u3);
		ja_right=ja;
		jb_right=jb;		
		jc_right=jc;		
		jd_right=jd;
		
		first_order_filter_cali(&lright_filter,l0);	
		l0=lright_filter.out;
		first_order_filter_cali(&aright_filter,a_right);	
		a_right=aright_filter.out;	
		first_order_filter_cali(&daright_filter,da_right);	
		da_right=daright_filter.out;	
		
		th_cos=arm_cos_f32(a_right);
		th_sin=arm_sin_f32(a_right);
		k=ja*jd-jb*jc;
		F_r=(jd*T_right2_r-jb*T_right1_r)*9.1f/k;
		Tp_r=(ja*T_right2_r-jc*T_right1_r)*9.1f/k;
		P=F_r*th_cos+Tp_r*th_sin/l0;
		Fn_right=P+mw*(AccelCorrected[1]+th_cos*da_right*da_right*l0+l0*th_sin*(da_right-da)*1000.0f+2*dl0_right*da_right*th_sin-th_cos*(dl0_right-l_dl0_right)*1000.0f);
		first_order_filter_cali(&Fright_filter,Fn_right);	
		Fn_right=Fright_filter.out;
		
		a=a_right;da=da_right;
		l0_right=l0;
		x=(x_left+x_right)/2.0f;
		dx=(dx_left+dx_right)/2.0f;
		//first_order_filter_cali(&x_filter,x);
		x=x_filter.out;//-l0*arm_sin_f32(u0);
		//first_order_filter_cali(&dx_filter,dx);
		dx=dx_filter.out;//-dl0_right*arm_sin_f32(u0)-l0_right*arm_cos_f32(da_right+db_right);			
//		a=a_right+0.008f;
		b=b_right;
		if(cal_flag==1)
				a=a_right+x*0.5;
		db=db_right;
		//x=(x_left+x_right)/2.0f;
		//dx=(dx_left+dx_right)/2.0f;
		//x_left=x;x_right=x;dx_left=dx;dx_right=dx;		
		l_u3_right=u3;
		ri=mw*(AccelCorrected[1]+th_cos*da_right*da_right*l0+l0*th_sin*(da_right-da)*1000.0f+2*dl0_right*da_right*th_sin-th_cos*(dl0_right-l_dl0_right)*1000.0f);
		//dl0_right=dl0_right*arm_cos_f32(a_right)+l0*arm_sin_f32(a_right)*da_right;
		l0*=arm_cos_f32(a_right);
		if(chassis_state==6)
		{
				F_right=360-360.0f/0.2f*(l0_right-0.1f);
		}
		else
		{
				if(chassis_state!=7)
						F_right=(lt_right-l0)*LEG_KP+i_l0_right-(l0-l_l0_right)*LEG_KD;
				else
						F_right=(lt_right-l0)*LEG_KP_DOWN-(l0-l_l0_right)*LEG_KD_DOWN;
				if(Chassis.Mode!=JUMP&&Chassis.Mode!=ROTING&&Chassis.Mode!=NOFORCE)
						i_l0_right+=(lt_right-l0)*LEG_KI;
				if(i_l0_right<0)		i_l0_right=0;
				if(i_l0_right>LEG_I_MAX)	i_l0_right=LEG_I_MAX;
				l_l0_right=l0;
				//if(F_right<-120)	F_right=-120;
				if(F_right>LEG_F_MAX)	F_right=LEG_F_MAX;
		}
		l0/=arm_cos_f32(a_right);
		if(Fn_right<10.0f)
		{
				count1++;
		}
		else
				count1=0;
//		if(count1>2&&Chassis.Mode!=JUMP)
//		{
//				T_right=0;
////				x_right=0;
//				xt_right=0;
//				Tp_right=(Calc_Poly(l0,0,0)*(a-kl*b_right)+Calc_Poly(l0,0,1)*da);
//		}
//		else
//		{
//				//count1=0;
////				if(x+xt_right>0.5f||x+xt_right<-0.5f||dx>0.5f||dx<-0.5f)
////				{
////						T_right=-(Calc_Poly_a(l0,0,0)*a+Calc_Poly_a(l0,0,1)*da+Calc_Poly_a(l0,0,2)*(x+xt_right)+Calc_Poly_a(l0,0,3)*(dx+Chassis.vx*1)+Calc_Poly_a(l0,0,4)*b+Calc_Poly_a(l0,0,5)*db);
////						Tp_right=-(Calc_Poly_a(l0,1,0)*a+Calc_Poly_a(l0,1,1)*da+Calc_Poly_a(l0,1,2)*(x+xt_right)+Calc_Poly_a(l0,1,3)*(dx+Chassis.vx*1)+Calc_Poly_a(l0,1,4)*b+Calc_Poly_a(l0,1,5)*db)-(a_right-a_left)*k0;
////				}
////				else
////				{
//						T_right=-(Calc_Poly(l0,0,0)*a+Calc_Poly(l0,0,1)*da+Calc_Poly(l0,0,2)*(x+xt_left)*(1-cal_flag)+Calc_Poly(l0,0,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0,0,4)*b+Calc_Poly(l0,0,5)*db);
//						Tp_right=-(Calc_Poly(l0,1,0)*a+Calc_Poly(l0,1,1)*da+Calc_Poly(l0,1,2)*(x+xt_left)*(1-cal_flag)+Calc_Poly(l0,1,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0,1,4)*b+Calc_Poly(l0,1,5)*db)-(a_right-a_left)*k0+(da_right-da_left)*d0;
////				}
//		}
////		if(chassis_state==5)
////		{
////				T_right1=-0.2;
////				T_right2=0.2;				
////		}
////		else
////		{
//				T_right1=-(ja*F_right+jb*Tp_right)/9.1f;
//				T_right2=-(jc*F_right+jd*Tp_right)/9.1f;
////		}



}

void Calculate_left()
{

		//float a,da,d2a,b,db,d2b,x,dx,d2x;	
		float ja,jb,jc,jd,k,th_sin,th_cos,F_r,Tp_r;
		first_order_filter_cali(&u1_left_filter,u1_left);	
		u1=u1_left_filter.out;
		first_order_filter_cali(&u4_left_filter,u4_left);	
		u4=u4_left_filter.out;	
		first_order_filter_cali(&du1_left_filter,du1_left);	
		du1=du1_left_filter.out;
		first_order_filter_cali(&du4_left_filter,du4_left);	
		du4=du4_left_filter.out;	
	
		cosu1=arm_cos_f32(u1_left),cosu4=arm_cos_f32(u4_left),sinu1=arm_sin_f32(u1_left),sinu4=arm_sin_f32(u4_left);

		xb=l1*cosu1-W/2.0f;
		yb=l1*sinu1;
		xd=W/2.0f+l1*cosu4;
		yd=l1*sinu4;
		lbd=Sqrt((xd-xb)*(xd-xb)+(yd-yb)*(yd-yb));
		A10=2*l2*(xd-xb);
		B10=2*l2*(yd-yb);
		C10=lbd*lbd;
		u2=2*atan2((B10+Sqrt(A10*A10+B10*B10-C10*C10)),(A10+C10));
		u3=atan2(yb-yd+l2*arm_sin_f32(u2),xb-xd+l2*arm_cos_f32(u2));
		sinu2u3=arm_sin_f32(u2-u3);
		xc=xb+l2*arm_cos_f32(u2);
		yc=yb+l2*arm_sin_f32(u2);
		u0=atan2(yc,xc);
		l0=Sqrt(xc*xc+yc*yc);
		dxc=l1*arm_sin_f32(u1-u2)*arm_sin_f32(u3)/sinu2u3*du1+l1*arm_sin_f32(u3-u4)*arm_sin_f32(u2)/sinu2u3*du4;
		dyc=l1*arm_sin_f32(u1-u2)*arm_cos_f32(u3)/sinu2u3*du1-l1*arm_sin_f32(u3-u4)*arm_cos_f32(u2)/sinu2u3*du4;
		d2xc_left=l1*arm_sin_f32(u1-u2)*arm_sin_f32(u3)/sinu2u3*d2u1_left+l1*arm_sin_f32(u3-u4)*arm_sin_f32(u2)/sinu2u3*d2u4_left;
		l_dl0_left=dl0_left;
		dl0_left=(xc*dxc+yc*dyc)/l0;
		first_order_filter_cali(&dlleft_filter,dl0_left);
		dl0_left=dlleft_filter.out;
		da_left=(dxc*yc-xc*dyc)/(xc*xc+yc*yc)-db_left;
		a_left=1.5708f-(b_left+u0);
		
		l0_left=l0;
		first_order_filter_cali(&lleft_filter,l0);	
		l0=lleft_filter.out;
		first_order_filter_cali(&aleft_filter,a_left);	
		a_left=aleft_filter.out;	
		first_order_filter_cali(&daleft_filter,da_left);	
		da_left=daleft_filter.out;	
		
		ja=l1*arm_sin_f32(u0-u3)*arm_sin_f32(u1-u2)/-sinu2u3;
		jb=l1*arm_cos_f32(u0-u3)*arm_sin_f32(u1-u2)/(l0*-sinu2u3);
		jc=l1*arm_sin_f32(u0-u2)*arm_sin_f32(u3-u4)/-sinu2u3;
		jd=l1*arm_cos_f32(u0-u2)*arm_sin_f32(u3-u4)/(l0*-sinu2u3);
		ja_left=ja;
		jb_left=jb;		
		jc_left=jc;	
		jd_left=jd;		
		
		
		th_cos=arm_cos_f32(a_left);
		th_sin=arm_sin_f32(a_left);
		k=ja*jd-jb*jc;
		F_r=(jd*T_left1_r-jb*T_left2_r)*9.1f/k;
		Tp_r=(ja*T_left1_r-jc*T_left2_r)*9.1f/k;
		P=F_r*th_cos+Tp_r*th_sin/l0;
		Fn_left=P+mw*(AccelCorrected[1]+th_cos*da_left*da_left*l0+l0*th_sin*(da_left-da)*1000.0f+2*dl0_left*da_left*th_sin-th_cos*(dl0_left-l_dl0_left)*1000.0f);
		first_order_filter_cali(&Fleft_filter,Fn_left);	
		Fn_left=Fleft_filter.out;
		
		
		a=a_left;da=da_left;

		x=(x_left+x_right)/2.0f;
		dx=(dx_left+dx_right)/2.0f;
		//first_order_filter_cali(&x_filter,x);
		x=x_filter.out;//-l0*arm_sin_f32(u0);
		//first_order_filter_cali(&dx_filter,dx);
		dx=dx_filter.out;//-dl0_left*arm_sin_f32(u0)-l0_left*arm_cos_f32(da_left+db_left);			
//		a=a_left+0.008f;
		b=b_left;
		if(cal_flag==1)
				a=a_left+x*0.5;
		db=db_left;
		//x_left=x;x_right=x;dx_left=dx;dx_right=dx;
		l_u3_left=u3;
		le=mw*(AccelCorrected[1]+th_cos*da_left*da_left*l0+l0*th_sin*(da_left-da)*1000.0f+2*dl0_left*da_left*th_sin-th_cos*(dl0_left-l_dl0_left)*1000.0f);
				
		//dl0_left=dl0_left*arm_cos_f32(a_left)+l0*arm_sin_f32(a_left)*da_left;
		l0*=arm_cos_f32(a_left);
		if(chassis_state==6)
		{
				F_left=360-360.0f/0.2f*(l0_left-0.1f);
		}
		else
		{
				if(chassis_state!=7)
						F_left=(lt_left-l0)*LEG_KP+i_l0_left-(l0-l_l0_left)*LEG_KD;
				else
						F_left=(lt_left-l0)*LEG_KP_DOWN-(l0-l_l0_left)*LEG_KD_DOWN;
				if(Chassis.Mode!=JUMP&&Chassis.Mode!=ROTING&&Chassis.Mode!=NOFORCE)
						i_l0_left+=(lt_left-l0)*LEG_KI;
				if(i_l0_left<0)		i_l0_left=0;
				if(i_l0_left>LEG_I_MAX)	i_l0_left=LEG_I_MAX;
				l_l0_left=l0;
				//if(F_left<-120)	F_left=-120;
				if(F_left>LEG_F_MAX)	F_left=LEG_F_MAX;
		}
		l0/=arm_cos_f32(a_left);
		if(Fn_left<10.0f)
		{
				count2++;
		}
		else
				count2=0;
//		if(count2>2&&Chassis.Mode!=JUMP)
//		{
//				T_left=0;
////				x_left=0;
//				xt_left=0;
//				Tp_left=(Calc_Poly(l0,0,0)*(a-kl*b_left)+Calc_Poly(l0,0,1)*da);
//		}
//		else
//		{
//				//count2=0;
////				if(x+xt_right>0.5f||x+xt_right<-0.5f||dx>0.5f||dx<-0.5f)
////				{
////						T_left=-(Calc_Poly_a(l0,0,0)*a+Calc_Poly_a(l0,0,1)*da+Calc_Poly_a(l0,0,2)*(x+xt_left)+Calc_Poly_a(l0,0,3)*(dx+Chassis.vx*1)+Calc_Poly_a(l0,0,4)*b+Calc_Poly_a(l0,0,5)*db);
////						Tp_left=-(Calc_Poly_a(l0,1,0)*a+Calc_Poly_a(l0,1,1)*da+Calc_Poly_a(l0,1,2)*(x+xt_left)+Calc_Poly_a(l0,1,3)*(dx+Chassis.vx*1)+Calc_Poly_a(l0,1,4)*b+Calc_Poly_a(l0,1,5)*db)+(a_right-a_left)*k0;
////				}
////				else
////				{
//						T_left=-(Calc_Poly(l0,0,0)*a+Calc_Poly(l0,0,1)*da+Calc_Poly(l0,0,2)*(x+xt_left)*(1-cal_flag)+Calc_Poly(l0,0,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0,0,4)*b+Calc_Poly(l0,0,5)*db);
//						Tp_left=-(Calc_Poly(l0,1,0)*a+Calc_Poly(l0,1,1)*da+Calc_Poly(l0,1,2)*(x+xt_left)*(1-cal_flag)+Calc_Poly(l0,1,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0,1,4)*b+Calc_Poly(l0,1,5)*db)+(a_right-a_left)*k0-(da_right-da_left)*d0;
//						//Pout=(a_right-a_left)*k0;
//						//Dout=(da_right-da_left)*d0;
////				}
//				}
//		if(chassis_state==5)
//		{
//				T_left1=0.2;
//				T_left2=-0.2;				
//		}
//		else
//		{
//				T_left1=(ja*F_left+jb*Tp_left)/9.1f;
//				T_left2=(jc*F_left+jd*Tp_left)/9.1f;
//		}


}

float T_left_test,Tp_left_test,T_right_test,Tp_right_test;
void Calculate_Kp()
{
		dx=(dx_left+dx_right)/2.0f;
		x=(x_left+x_right)/2.0f;
		dxt_left=Chassis.vx*0.1f+Chassis.wz*0.5/2*0.1f;//+l0_left*arm_cos_f32(a_left)*da_left;
		dxt_right=Chassis.vx*0.1f-Chassis.wz*0.5/2*0.1f;//+l0_right*arm_cos_f32(a_right)*da_right;
		xt_left+=Chassis.vx*0.001f+Chassis.wz*0.5/2*0.001f;
		xt_right+=Chassis.vx*0.001f-Chassis.wz*0.5/2*0.001f;
		x_left+=dx_left*0.001f;	
		x_right+=dx_right*0.001f;	
		first_order_filter_cali(&dxleft_filter,dx_left);
		first_order_filter_cali(&dxright_filter,dx_right);
		first_order_filter_cali(&xleft_filter,x_left);
		first_order_filter_cali(&xright_filter,x_right);		
	
		if((count2>2||count1>2)&&Chassis.Mode!=JUMP)
		{
				T_left=0;
				x_left=0;
				xt_left=0;
				Tp_left=-(Calc_Poly_extern(l0_left,1,0)*a_left+Calc_Poly_extern(l0_left,1,1)*da_left+Calc_Poly_extern(l0_right,0,2)*a_right+Calc_Poly_extern(l0_right,0,3)*da_right);
				T_right=0;
				x_right=0;
				xt_right=0;
				Tp_right=-(Calc_Poly_extern(l0_left,2,0)*a_left+Calc_Poly_extern(l0_left,2,1)*da_left+Calc_Poly_extern(l0_right,3,2)*a_right+Calc_Poly_extern(l0_right,3,3)*da_right);
		}
		else
		{

				T_left=-(Calc_Poly_extern(l0_left,0,0)*a_left+Calc_Poly_extern(l0_left,0,1)*da_left+Calc_Poly_extern(l0_left,0,4)*(xleft_filter.out+xt_left)*(1-cal_flag)+Calc_Poly_extern(l0_left,0,5)*(dxleft_filter.out+dxt_left)+Calc_Poly_extern(l0_left,0,8)*b_left+Calc_Poly_extern(l0_left,0,9)*db_left
			+Calc_Poly_extern(l0_right,0,2)*a_right+Calc_Poly_extern(l0_right,0,3)*da_right+Calc_Poly_extern(l0_right,0,6)*(xright_filter.out+xt_right)*(1-cal_flag)+Calc_Poly_extern(l0_right,0,7)*(dxright_filter.out+dxt_right));
				Tp_left=-(Calc_Poly_extern(l0_left,1,0)*a_left+Calc_Poly_extern(l0_left,1,1)*da_left+Calc_Poly_extern(l0_left,1,4)*(xleft_filter.out+xt_left)*(1-cal_flag)+Calc_Poly_extern(l0_left,1,5)*(dxleft_filter.out+dxt_left)+Calc_Poly_extern(l0_left,1,8)*b_left+Calc_Poly_extern(l0_left,1,9)*db_left
			+Calc_Poly_extern(l0_right,1,2)*a_right+Calc_Poly_extern(l0_right,1,3)*da_right+Calc_Poly_extern(l0_right,1,6)*(xright_filter.out+xt_right)*(1-cal_flag)+Calc_Poly_extern(l0_right,1,7)*(dxright_filter.out+dxt_right));
				T_right=-(Calc_Poly_extern(l0_left,2,0)*a_left+Calc_Poly_extern(l0_left,2,1)*da_left+Calc_Poly_extern(l0_left,2,4)*(xleft_filter.out+xt_left)*(1-cal_flag)+Calc_Poly_extern(l0_left,2,5)*(dxleft_filter.out+dxt_left)+Calc_Poly_extern(l0_right,2,8)*b_right+Calc_Poly_extern(l0_right,2,9)*db_right
			+Calc_Poly_extern(l0_right,2,2)*a_right+Calc_Poly_extern(l0_right,2,3)*da_right+Calc_Poly_extern(l0_right,2,6)*(xright_filter.out+xt_right)*(1-cal_flag)+Calc_Poly_extern(l0_right,2,7)*(dxright_filter.out+dxt_right));
				Tp_right=-(Calc_Poly_extern(l0_left,3,0)*a_left+Calc_Poly_extern(l0_left,3,1)*da_left+Calc_Poly_extern(l0_left,3,4)*(xleft_filter.out+xt_left)*(1-cal_flag)+Calc_Poly_extern(l0_left,3,5)*(dxleft_filter.out+dxt_left)+Calc_Poly_extern(l0_right,3,8)*b_right+Calc_Poly_extern(l0_right,3,9)*db_right
			+Calc_Poly_extern(l0_right,3,2)*a_right+Calc_Poly_extern(l0_right,3,3)*da_right+Calc_Poly_extern(l0_right,3,6)*(xright_filter.out+xt_right)*(1-cal_flag)+Calc_Poly_extern(l0_right,3,7)*(dxright_filter.out+dxt_right));

//				T_left=-(Calc_Poly(l0_left,0,0)*a_left+Calc_Poly(l0_left,0,1)*da_left+Calc_Poly(l0_left,0,2)*(x+xt_left)*(1-cal_flag)+Calc_Poly(l0_left,0,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0_left,0,4)*b_left+Calc_Poly(l0_left,0,5)*db_left);
//				Tp_left=-(Calc_Poly(l0_left,1,0)*a_left+Calc_Poly(l0_left,1,1)*da_left+Calc_Poly(l0_left,1,2)*(x+xt_left)*(1-cal_flag)+Calc_Poly(l0_left,1,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0_left,1,4)*b_left+Calc_Poly(l0_left,1,5)*db_left);
//				T_right=-(Calc_Poly(l0_right,0,0)*a_right+Calc_Poly(l0_right,0,1)*da_right+Calc_Poly(l0_right,0,2)*(x+xt_right)*(1-cal_flag)+Calc_Poly(l0_right,0,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0_right,0,4)*b_right+Calc_Poly(l0_right,0,5)*db_right);
//				Tp_right=-(Calc_Poly(l0_right,1,0)*a_right+Calc_Poly(l0_right,1,1)*da_right+Calc_Poly(l0_right,1,2)*(x+xt_right)*(1-cal_flag)+Calc_Poly(l0_right,1,3)*(dx+Chassis.vx*0.5)+Calc_Poly(l0_right,1,4)*b_right+Calc_Poly(l0_right,1,5)*db_right);
//			
		}
		T_left1=(ja_left*F_left+jb_left*Tp_left)/9.1f;
		T_left2=(jc_left*F_left+jd_left*Tp_left)/9.1f;
		T_right1=-(ja_right*F_right+jb_right*Tp_right)/9.1f;
		T_right2=-(jc_right*F_right+jd_right*Tp_right)/9.1f;

}