#ifndef CONTROL_H
#define CONTROL_H

#include "CalculateThread.h"
#include "feet_motor.h"
#include "Remote.h"
#include "AttitudeThread.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Setting.h"
#include "joint_motor.h"
#include "struct_typedef.h"
#include "stdio.h"
#include "crc32.h"
#include "user_lib.h"
#include "stm32h7xx_it.h"

typedef enum{
	LEFT1,
	LEFT2,
	RIGHT1,
	RIGHT2
}ID;

extern first_order_filter_type_t xleft_filter,xright_filter,dxleft_filter,dxright_filter;
extern float T_left1,T_left2,T_right1,T_right2,T_right,T_left,T_left1_r,T_left2_r,T_right1_r,T_right2_r;
extern float a_right,da_right,x_right,dx_right,b_right,db_right,last_a_right,a_left,da_left,x_left,dx_left,b_left,db_left,last_a_left;
extern float T,Tp_left,Tp_right,N,P,Nm,Pm,R,g,L,Lm,l,mw,mp,M,Iw,Ip,Im;	
extern float u1_right,u4_right,du1_right,du4_right,u1_left,u4_left,du1_left,du4_left;
extern float KLEFT1,KLEFT2,KRIGHT1,KRIGHT2;
extern float d2u1_right,d2u4_right,d2u1_left,d2u4_left;
extern float l0_left,l0_right,dl0_left,dl0_right;
extern float a,da,d2a,b,db,d2b,x,dx,d2x,Fn_left,Fn_right;
extern float count1,count2;
extern void Calculate_Init();
extern void ChassisStateUpdate();
extern void ChassisModeUpdate();
extern void Calculate_LQR();
extern float Calc_pos(float pos,ID id);
extern void Change(float pos,ID id);
extern void Calculate_right(void);
extern void Calculate_left(void);
extern void Calculate_Kp(void);
#endif
