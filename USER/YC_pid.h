/*
 * my_pid.h
 *
 *  Created on: 2018��5��11��
 *      Author: wyp
 */

#ifndef USER_MY_PID_H_
#define USER_MY_PID_H_

#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdlib.h"
#include "main.h"
#include "math.h"
//#include "..\DSP\Include\arm_common_tables.h"
//#include "..\DSP\Include\arm_const_structs.h"
//#include "..\DSP\Include\arm_math.h"



#define PID_INT_EN      0           //pid�Ķ���(int)����ʹ��
#define PID_FLO_EN      1           //pid�ĸ���(float)����ʹ��


#ifndef PID_DATA_TYPES
#define PID_DATA_TYPES

#if PID_INT_EN  ==1
typedef int         PID_DataType;
#endif

#if PID_FLO_EN  == 1
typedef float       PID_DataType;
#endif

#endif




typedef struct  _PID{

    PID_DataType    T;
    PID_DataType    Kp;
    PID_DataType    Ti;
    PID_DataType    Td;

    PID_DataType    a0;
    PID_DataType    a1;
    PID_DataType    a2;

    PID_DataType    Ek_0;
    PID_DataType    Ek_1;
    PID_DataType    Ek_2;
	
		PID_DataType    DDD_NOW;     //��¼�ϴε�΢�������������ʵ�ֲ���ȫ΢��	
		PID_DataType    DDD_LAST;     //��¼�ϴε�΢�������������ʵ�ֲ���ȫ΢��
	
	
    PID_DataType    Inc;
    PID_DataType    N_Output;
    PID_DataType    L_Output;

    PID_DataType    IntegroMax;
    PID_DataType    IntegroMin;
    PID_DataType    OutMax;
    PID_DataType    OutMin;

}PID_TypeDef;


//extern PID_TypeDef pid_voltage_loop;   //��ѹ��PID
//extern PID_TypeDef pid_current_loop;   //������PID
//extern PID_TypeDef pid_HV_loop;   //��ѹг�����PID
extern PID_TypeDef pid_GM6020_ID6_Speed_loop;


extern PID_DataType Inc_pid_calc ( PID_TypeDef *p, PID_DataType ref, PID_DataType fb);
extern void Loc_pid_reset( PID_TypeDef *p);
extern void Loc_pid_init ( PID_TypeDef *p);
extern void Loc_pid_clc  ( PID_TypeDef *p);

extern void Inc_pid_reset( PID_TypeDef *p);
extern void Inc_pid_init ( PID_TypeDef *p);
extern void Inc_pid_clc  ( PID_TypeDef *p);


extern PID_DataType Loc_pid_calc( PID_TypeDef *p, PID_DataType ref, PID_DataType fb);
extern PID_DataType Loc_pid_calc_2( PID_TypeDef *p,PID_DataType Irate, PID_DataType ref, PID_DataType fb);
extern PID_DataType Loc_pid_calc_3( PID_TypeDef *p,PID_DataType Irate, PID_DataType ref, PID_DataType fb);
extern void PID_PARA_INIT(void);

float  type3_clc(float vout_temp,float vout_ref);//TYPE3�㷨
float  PID_BM_clc(float Vset_target,float Vout_actual) ;    //������PID
void Loc_pid_calc_rest( PID_TypeDef *p);// �ָ���ʼֵ


#endif /* USER_MY_PID_H_ */
