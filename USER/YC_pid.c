/*
 * my_pid.c
 *
 *  Created on: 2018年5月11日
 *      Author: wyp
 */

#include "YC_pid.h"

//PID_TypeDef pid_voltage_loop;   //电压环PID
//PID_TypeDef pid_current_loop;   //电流环PID
//PID_TypeDef pid_HV_loop;   //高压谐振输出PID

PID_TypeDef pid_GM6020_ID6_Speed_loop;   //ID为6的GM6020电机速度控制

void PID_PARA_INIT(void)
{
	float temp;
	
    Inc_pid_reset(&pid_GM6020_ID6_Speed_loop);
    pid_GM6020_ID6_Speed_loop.T          = 1.0f * 1e6 / 1e3;//65kHz(15.38us)调整一次
    pid_GM6020_ID6_Speed_loop.Kp         = 30.0f;//10
    pid_GM6020_ID6_Speed_loop.Ti         = 3000.0f;//800
    pid_GM6020_ID6_Speed_loop.Td         = 0.00000005f;
	

    pid_GM6020_ID6_Speed_loop.OutMax =  30000;//最大的电压给定值
		pid_GM6020_ID6_Speed_loop.OutMin =  -30000;//最小电压给定值
    Inc_pid_init(&pid_GM6020_ID6_Speed_loop);
	
	
	
	
}

//增量式PID复位值函数
void Inc_pid_reset( PID_TypeDef *p)
{
    p->a0           = 0;//差分方程的系数
    p->a1           = 0;
    p->a2           = 0;

    p->Ek_0     = 0;     //  Error[k]
    p->Ek_1     = 0;     //  Error[k-1]
    p->Ek_2     = 0;     //  Error[k-2]

    p->T            = 0;
    p->Kp           = 0;
    p->Ti           = 0;
    p->Td           = 0;

    p->L_Output = 0;
    p->N_Output = 0;

    p->Inc          = 0;

    p->OutMax       = 0;
    p->OutMin       = 0;
}

//增量式PID初始化函数
void Inc_pid_init ( PID_TypeDef *p)
{

    if(p->Ti == 0)
    {
        p->Ti       = p->Ti == 0 ? (PID_DataType)0xFFFFFFFF: p->Ti;
    }

		//计算差分方程的系数
    p->a0       = p->Kp*(1 + 1.0f*p->T/p->Ti + 1.0f*p->Td/p->T);
    p->a1       = p->Kp*(1 + 2.0f*p->Td/p->T);
    p->a2       = 1.0f*p->Kp*p->Td/p->T;
}

//增量式PID计算函数
//#pragma arm section code = "RAMCODE"
PID_DataType Inc_pid_calc( PID_TypeDef *p, PID_DataType ref, PID_DataType fb)
{
    p->Ek_0     = ref - fb; //计算误差

    p->Inc      = (     p->a0 * p->Ek_0 \
                              - p->a1 * p->Ek_1 \
                  + p->a2 * p->Ek_2 );          //PID增量式计算

    p->N_Output = p->L_Output + p->Inc;//PID增量式计算

    if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;
    if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

    p->Ek_2        = p->Ek_1;
    p->Ek_1        = p->Ek_0;
    p->L_Output    = p->N_Output;

    return p->N_Output;

}

//增量式PID清空值函数
void Inc_pid_clc( PID_TypeDef   *p)
{
    p->Ek_0             = 0;     //  Error[k]
    p->Ek_1             = 0;     //  Error[k-1]
    p->Ek_2             = 0;     //  Error[k-2]
    p->Inc              = 0;
    p->L_Output     = 0;
    p->N_Output     = 0;
}

//位置式PID复位函数
void Loc_pid_reset( PID_TypeDef *p)
{
    p->a0           = 0;
    p->a1           = 0;
    p->a2           = 0;

    p->Ek_0     = 0;     //  Error[k]
    p->Ek_1     = 0;     //  Error[k-1]
    p->Ek_2     = 0;     //  Sum_Error

    p->T            = 0;
    p->Kp           = 0;
    p->Ti           = 0;
    p->Td           = 0;

    p->IntegroMax = 0;
    p->IntegroMin = 0;

    p->L_Output = 0;
    p->N_Output = 0;

    p->Inc          = 0;

    p->OutMax       = 0;
    p->OutMin       = 0;
}

//位置式PID初始化函数
void Loc_pid_init ( PID_TypeDef *p)
{

    if(p->Ti == 0)
    {
        p->Ti       = p->Ti == 0 ? (PID_DataType)0xFFFFFFFF: p->Ti;
    }

    p->a0       = p->Kp;//计算差分方程系数
    p->a1       = p->Kp*(p->T/p->Ti);
    p->a2       = p->Kp*(p->Td/p->T);

    p->IntegroMax =p->IntegroMax / p->a1;
    p->IntegroMin = -1.0f * p->IntegroMax;
}

//位置式PID计算函数
//#pragma arm section code = "RAMCODE"
PID_DataType Loc_pid_calc( PID_TypeDef *p, PID_DataType ref, PID_DataType fb)
{
	static float EC;//定义误差的变化率
	static float ABS_E;//定义误差的绝对值
	static float III_temp;//这个变量在Loc_pid_calc函数中使用，定义成全局变量的目的是可以在仿真的时候实时观察到这个变量的变化（如果在函数内部定义，则在仿真时不便于观察）	
	
//	if(ref<50)
//		ref =50;
	
	p->Ek_0	= ref - fb; //计算误差，误差等于设定值与实际测量值的偏差

	//p->Ek_0表示当前的误差，p->Ek_1表示上次的误差
	EC = p->Ek_0 - p->Ek_1;//当前误差减去上次误差，如果大于0，表示误差在变大	
	
	p->DDD_NOW = p->a2 * EC *0.2f + p->DDD_LAST *0.8f; //不完全微分，p->a2代表微分系数Kd
	p->DDD_LAST = p->DDD_NOW;	
	
	//积分分离PID，抗饱和积分
//	ABS_E = fabsf(p->Ek_0);
//	if(ABS_E < 100)//如果当前误差在100以内，就正常进行积分
//	{
//		III_temp = 1.0f;
//	}
//	else if(ABS_E <1000)//如果当前误差在100-1100之内，就线性改变积分累计的增益系数1  -  0.01
//	{
//		III_temp = 1.0f - 0.00099f * (ABS_E - 100);
//	}	
//	else if(ABS_E <3000)//如果当前误差在1100-3000之内，积分累积的系数恒为0.000001
//	{
//		III_temp = 0.000001f;
//	}		
//	else//如果当前误差大于3000，就完全取消积分作用
//	{
//		III_temp = 0.000f;
//	}	
//	
////	p->Ek_2        = p->Ek_2 + p->Ek_0*III_temp;
//if(ABS_E > 20)//如果当前误差在20以内，就正常进行积分
//	p->Ek_2        = p->Ek_2;
//else
	p->Ek_2        = p->Ek_2 + p->Ek_0;
	
	if( p->Ek_2 > p->IntegroMax) p->Ek_2   =  p->IntegroMax;//积分限幅
	if( p->Ek_2 < p->IntegroMin) p->Ek_2   =  p->IntegroMin;		

	p->N_Output      = (            p->a0 * p->Ek_0 \
														+     p->a1 * p->Ek_2 \
														+     p->DDD_NOW );          //PID计算

//	p->N_Output      = (            p->a0 * p->Ek_0 \
//														+     p->a1 * p->Ek_2 );          //PID计算	

	
	p->N_Output = p->N_Output * 0.8f + p->L_Output * 0.2f;//LPF

	if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;//PID输出限幅
	if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

	p->Ek_1        = p->Ek_0;//更新历史值
	p->L_Output    = p->N_Output;

	return p->N_Output;
}

///位置式PID计算函数
//#pragma arm section code = "RAMCODE"
PID_DataType Loc_pid_calc_2( PID_TypeDef *p,PID_DataType Irate, PID_DataType ref, PID_DataType fb)
{
	static float EC;//定义误差的变化率
	static float ABS_E;//定义误差的绝对值
	static float III_temp;//这个变量在Loc_pid_calc函数中使用，定义成全局变量的目的是可以在仿真的时候实时观察到这个变量的变化（如果在函数内部定义，则在仿真时不便于观察）	
	
//	if(ref<50)
//		ref =50;
	
	p->Ek_0	= ref - fb; //计算误差，误差等于设定值与实际测量值的偏差

	//p->Ek_0表示当前的误差，p->Ek_1表示上次的误差
	EC = p->Ek_0 - p->Ek_1;//当前误差减去上次误差，如果大于0，表示误差在变大	
	
//	p->DDD_NOW = p->a2 * EC *0.2f + p->DDD_LAST *0.8f; //不完全微分，p->a2代表微分系数Kd
//	p->DDD_LAST = p->DDD_NOW;	
	
	//积分分离PID，抗饱和积分
//	ABS_E = fabsf(p->Ek_0);
//	if(ABS_E < 100)//如果当前误差在100以内，就正常进行积分
//	{
//		III_temp = 1.0f;
//	}
//	else if(ABS_E <1000)//如果当前误差在100-1100之内，就线性改变积分累计的增益系数1  -  0.01
//	{
//		III_temp = 1.0f - 0.00099f * (ABS_E - 100);
//	}	
//	else if(ABS_E <3000)//如果当前误差在1100-3000之内，积分累积的系数恒为0.000001
//	{
//		III_temp = 0.000001f;
//	}		
//	else//如果当前误差大于3000，就完全取消积分作用
//	{
//		III_temp = 0.000f;
//	}	
//	
////	p->Ek_2        = p->Ek_2 + p->Ek_0*III_temp;
//if(ABS_E > 20)//如果当前误差在20以内，就正常进行积分
//	p->Ek_2        = p->Ek_2;
//else
	p->Ek_2        = p->Ek_2 + p->Ek_0 * Irate;
	
	if( p->Ek_2 > p->IntegroMax * 0.9f) p->Ek_2   =  p->IntegroMax * 0.9f;//积分限幅
	if( p->Ek_2 < p->IntegroMin * 0.9f) p->Ek_2   =  p->IntegroMin * 0.9f;		

	p->N_Output      = (            p->a0 * p->Ek_0 * 1 \
														+     p->a1 * p->Ek_2 \
														);          //PID计算

//	p->N_Output      = (            p->a0 * p->Ek_0 \
//														+     p->a1 * p->Ek_2 );          //PID计算	

	
	p->N_Output = p->N_Output * 0.3f + p->L_Output * 0.7f;//LPF

	if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;//PID输出限幅
	if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

	p->Ek_1        = p->Ek_0;//更新历史值
	p->L_Output    = p->N_Output;

	return p->N_Output;
}

PID_DataType Loc_pid_calc_3( PID_TypeDef *p,PID_DataType Irate, PID_DataType ref, PID_DataType fb)
{
	static float EC;//定义误差的变化率
	static float ABS_E;//定义误差的绝对值
	static float III_temp;//这个变量在Loc_pid_calc函数中使用，定义成全局变量的目的是可以在仿真的时候实时观察到这个变量的变化（如果在函数内部定义，则在仿真时不便于观察）	
	
//	if(ref<50)
//		ref =50;
	
	p->Ek_0	= ref - fb; //计算误差，误差等于设定值与实际测量值的偏差

	//p->Ek_0表示当前的误差，p->Ek_1表示上次的误差
	EC = p->Ek_0 - p->Ek_1;//当前误差减去上次误差，如果大于0，表示误差在变大	
	
	p->DDD_NOW = p->a2 * EC * 300 *0.06f + p->DDD_LAST *0.94f; //不完全微分，p->a2代表微分系数Kd
	p->DDD_LAST = p->DDD_NOW;	
	
	//积分分离PID，抗饱和积分
//	ABS_E = fabsf(p->Ek_0);
//	if(ABS_E < 100)//如果当前误差在100以内，就正常进行积分
//	{
//		III_temp = 1.0f;
//	}
//	else if(ABS_E <1000)//如果当前误差在100-1100之内，就线性改变积分累计的增益系数1  -  0.01
//	{
//		III_temp = 1.0f - 0.00099f * (ABS_E - 100);
//	}	
//	else if(ABS_E <3000)//如果当前误差在1100-3000之内，积分累积的系数恒为0.000001
//	{
//		III_temp = 0.000001f;
//	}		
//	else//如果当前误差大于3000，就完全取消积分作用
//	{
//		III_temp = 0.000f;
//	}	
//	
////	p->Ek_2        = p->Ek_2 + p->Ek_0*III_temp;
//if(ABS_E > 20)//如果当前误差在20以内，就正常进行积分
//	p->Ek_2        = p->Ek_2;
//else
	p->Ek_2        = p->Ek_2 + p->Ek_0 * Irate;
	
	if( p->Ek_2 > p->IntegroMax * 1.0f) p->Ek_2   =  p->IntegroMax * 1.0f;//积分限幅
	if( p->Ek_2 < p->IntegroMin * 1.0f) p->Ek_2   =  p->IntegroMin * 1.0f;		


	p->N_Output      = (            p->a0 * p->Ek_0 * 1 \
														+     p->a1 * p->Ek_2 \
														+     p->DDD_NOW );          //PID计算

//	p->N_Output      = (            p->a0 * p->Ek_0 * 0.01f \
//														+     p->a1 * p->Ek_2 \
//														);          //PID计算

//	p->N_Output      = (            p->a0 * p->Ek_0 \
//														+     p->a1 * p->Ek_2 );          //PID计算	

	
	p->N_Output = p->N_Output * 0.1f + p->L_Output * 0.9f;//LPF

	if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;//PID输出限幅
	if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

	p->Ek_1        = p->Ek_0;//更新历史值
	p->L_Output    = p->N_Output;

	return p->N_Output;
}


//位置式PID清除数据函数
void Loc_pid_clc( PID_TypeDef   *p)
{
    p->Ek_0             = 0;     //  Error[k]
    p->Ek_1             = 0;     //  Error[k-1]
    p->Ek_2             = 0;     //  Error[k-2]
    p->L_Output     = 0;
    p->N_Output     = 0;
}


 float      Xn,Xn_1,Xn_2,Xn_3;   //差分方程输入项：输出电压-参考电压
 float      Yn,Yn_1,Yn_2,Yn_3;   //差分方程输出项：误差放大器输出Verror
 float kx0=136.152858f;            //差分方程系数：对应差分方程各项  OK的
 float kx1=-2.807968f;
 float kx2=-0.620162f;
 float kx3=112.212783f;
 float ky1=-0.811847f;
 float ky2=-0.179303f;
 float ky3=-0.008850f;
float  type3_clc(float vout_ref,float vout_temp)//TYPE3算法
{
	 Xn=(float)(vout_ref-vout_temp)*0.0002f;          //参考电压-输出电压
	 Yn=kx0*Xn+kx1*Xn_1+kx2*Xn_2+kx3*Xn_3-ky1*Yn_1-ky2*Yn_2-ky3*Yn_3; //这个方程代表模拟电路type3补偿器

	 if(Yn>=1499){Yn=1499;}					     //最大占空比关95%                          
	 else if(Yn<=0.00f){Yn=0.00f;}              //最小占空比0.0%
	 
	 Xn_3=Xn_2;Xn_2=Xn_1;Xn_1=Xn;							 //移位				 														 
	 Yn_3=Yn_2;Yn_2=Yn_1;Yn_1=Yn;	
	 return Yn;	 	 
}


 float        Kp_V=120.0f;             //比例系数
 float        Ki_V=0.3f;             //积分系数
 float        Kd_V=50.0f;             //微分系数
 float     	 error1_V=0.0f;         //当前输出和设定值误差
 float      	 error2_V=0.0f;         //当前输出和设定值误差
 float      	 e1_V=0.0f;             //上次输出和设定值误差
 float      	 e2_V=0.0f;             //上上次输出和设定值误差
 

 __IO uint16_t Pulse_width=0;
float PID_BM_clc(float Vset_target,float Vout_actual)     //增量型PID
{ 
	static float Yn=0;
	//计算电压PID环路
	error1_V=(Vset_target-Vout_actual);                       //当前误差
	e1_V=error1_V-error2_V;                                          //两次误差变化量
	Yn+=Kp_V*e1_V+Ki_V*error1_V+Kd_V*(e1_V-e2_V);      //优化之后的PID算法(Kp*(ek-ek_1)+Ki*ek+Kd*(ek-2ek_1+ek_2))
	error2_V=error1_V;e2_V=e1_V;                                     //移位
	
	//if(error1_V<-0.2f)ERROR_Increase_V-=0.1f;
	//限幅
	if(Yn>=1499)Yn=1499.0f;                
	else if(Yn<=0.00f)Yn=0.0f;
  return Yn;	
	
}



void Loc_pid_calc_rest( PID_TypeDef *p)
{
	p->Ek_0	= 0;
	p->Ek_1	= 0;
	p->Ek_2 =0;
	
	p->DDD_NOW =0;
	p->DDD_LAST =0;	

	p->N_Output =0;
	p->L_Output=0 ;
}



