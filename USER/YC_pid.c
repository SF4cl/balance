/*
 * my_pid.c
 *
 *  Created on: 2018��5��11��
 *      Author: wyp
 */

#include "YC_pid.h"

//PID_TypeDef pid_voltage_loop;   //��ѹ��PID
//PID_TypeDef pid_current_loop;   //������PID
//PID_TypeDef pid_HV_loop;   //��ѹг�����PID

PID_TypeDef pid_GM6020_ID6_Speed_loop;   //IDΪ6��GM6020����ٶȿ���

void PID_PARA_INIT(void)
{
	float temp;
	
    Inc_pid_reset(&pid_GM6020_ID6_Speed_loop);
    pid_GM6020_ID6_Speed_loop.T          = 1.0f * 1e6 / 1e3;//65kHz(15.38us)����һ��
    pid_GM6020_ID6_Speed_loop.Kp         = 30.0f;//10
    pid_GM6020_ID6_Speed_loop.Ti         = 3000.0f;//800
    pid_GM6020_ID6_Speed_loop.Td         = 0.00000005f;
	

    pid_GM6020_ID6_Speed_loop.OutMax =  30000;//���ĵ�ѹ����ֵ
		pid_GM6020_ID6_Speed_loop.OutMin =  -30000;//��С��ѹ����ֵ
    Inc_pid_init(&pid_GM6020_ID6_Speed_loop);
	
	
	
	
}

//����ʽPID��λֵ����
void Inc_pid_reset( PID_TypeDef *p)
{
    p->a0           = 0;//��ַ��̵�ϵ��
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

//����ʽPID��ʼ������
void Inc_pid_init ( PID_TypeDef *p)
{

    if(p->Ti == 0)
    {
        p->Ti       = p->Ti == 0 ? (PID_DataType)0xFFFFFFFF: p->Ti;
    }

		//�����ַ��̵�ϵ��
    p->a0       = p->Kp*(1 + 1.0f*p->T/p->Ti + 1.0f*p->Td/p->T);
    p->a1       = p->Kp*(1 + 2.0f*p->Td/p->T);
    p->a2       = 1.0f*p->Kp*p->Td/p->T;
}

//����ʽPID���㺯��
//#pragma arm section code = "RAMCODE"
PID_DataType Inc_pid_calc( PID_TypeDef *p, PID_DataType ref, PID_DataType fb)
{
    p->Ek_0     = ref - fb; //�������

    p->Inc      = (     p->a0 * p->Ek_0 \
                              - p->a1 * p->Ek_1 \
                  + p->a2 * p->Ek_2 );          //PID����ʽ����

    p->N_Output = p->L_Output + p->Inc;//PID����ʽ����

    if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;
    if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

    p->Ek_2        = p->Ek_1;
    p->Ek_1        = p->Ek_0;
    p->L_Output    = p->N_Output;

    return p->N_Output;

}

//����ʽPID���ֵ����
void Inc_pid_clc( PID_TypeDef   *p)
{
    p->Ek_0             = 0;     //  Error[k]
    p->Ek_1             = 0;     //  Error[k-1]
    p->Ek_2             = 0;     //  Error[k-2]
    p->Inc              = 0;
    p->L_Output     = 0;
    p->N_Output     = 0;
}

//λ��ʽPID��λ����
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

//λ��ʽPID��ʼ������
void Loc_pid_init ( PID_TypeDef *p)
{

    if(p->Ti == 0)
    {
        p->Ti       = p->Ti == 0 ? (PID_DataType)0xFFFFFFFF: p->Ti;
    }

    p->a0       = p->Kp;//�����ַ���ϵ��
    p->a1       = p->Kp*(p->T/p->Ti);
    p->a2       = p->Kp*(p->Td/p->T);

    p->IntegroMax =p->IntegroMax / p->a1;
    p->IntegroMin = -1.0f * p->IntegroMax;
}

//λ��ʽPID���㺯��
//#pragma arm section code = "RAMCODE"
PID_DataType Loc_pid_calc( PID_TypeDef *p, PID_DataType ref, PID_DataType fb)
{
	static float EC;//�������ı仯��
	static float ABS_E;//�������ľ���ֵ
	static float III_temp;//���������Loc_pid_calc������ʹ�ã������ȫ�ֱ�����Ŀ���ǿ����ڷ����ʱ��ʵʱ�۲쵽��������ı仯������ں����ڲ����壬���ڷ���ʱ�����ڹ۲죩	
	
//	if(ref<50)
//		ref =50;
	
	p->Ek_0	= ref - fb; //�������������趨ֵ��ʵ�ʲ���ֵ��ƫ��

	//p->Ek_0��ʾ��ǰ����p->Ek_1��ʾ�ϴε����
	EC = p->Ek_0 - p->Ek_1;//��ǰ����ȥ�ϴ����������0����ʾ����ڱ��	
	
	p->DDD_NOW = p->a2 * EC *0.2f + p->DDD_LAST *0.8f; //����ȫ΢�֣�p->a2����΢��ϵ��Kd
	p->DDD_LAST = p->DDD_NOW;	
	
	//���ַ���PID�������ͻ���
//	ABS_E = fabsf(p->Ek_0);
//	if(ABS_E < 100)//�����ǰ�����100���ڣ����������л���
//	{
//		III_temp = 1.0f;
//	}
//	else if(ABS_E <1000)//�����ǰ�����100-1100֮�ڣ������Ըı�����ۼƵ�����ϵ��1  -  0.01
//	{
//		III_temp = 1.0f - 0.00099f * (ABS_E - 100);
//	}	
//	else if(ABS_E <3000)//�����ǰ�����1100-3000֮�ڣ������ۻ���ϵ����Ϊ0.000001
//	{
//		III_temp = 0.000001f;
//	}		
//	else//�����ǰ������3000������ȫȡ����������
//	{
//		III_temp = 0.000f;
//	}	
//	
////	p->Ek_2        = p->Ek_2 + p->Ek_0*III_temp;
//if(ABS_E > 20)//�����ǰ�����20���ڣ����������л���
//	p->Ek_2        = p->Ek_2;
//else
	p->Ek_2        = p->Ek_2 + p->Ek_0;
	
	if( p->Ek_2 > p->IntegroMax) p->Ek_2   =  p->IntegroMax;//�����޷�
	if( p->Ek_2 < p->IntegroMin) p->Ek_2   =  p->IntegroMin;		

	p->N_Output      = (            p->a0 * p->Ek_0 \
														+     p->a1 * p->Ek_2 \
														+     p->DDD_NOW );          //PID����

//	p->N_Output      = (            p->a0 * p->Ek_0 \
//														+     p->a1 * p->Ek_2 );          //PID����	

	
	p->N_Output = p->N_Output * 0.8f + p->L_Output * 0.2f;//LPF

	if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;//PID����޷�
	if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

	p->Ek_1        = p->Ek_0;//������ʷֵ
	p->L_Output    = p->N_Output;

	return p->N_Output;
}

///λ��ʽPID���㺯��
//#pragma arm section code = "RAMCODE"
PID_DataType Loc_pid_calc_2( PID_TypeDef *p,PID_DataType Irate, PID_DataType ref, PID_DataType fb)
{
	static float EC;//�������ı仯��
	static float ABS_E;//�������ľ���ֵ
	static float III_temp;//���������Loc_pid_calc������ʹ�ã������ȫ�ֱ�����Ŀ���ǿ����ڷ����ʱ��ʵʱ�۲쵽��������ı仯������ں����ڲ����壬���ڷ���ʱ�����ڹ۲죩	
	
//	if(ref<50)
//		ref =50;
	
	p->Ek_0	= ref - fb; //�������������趨ֵ��ʵ�ʲ���ֵ��ƫ��

	//p->Ek_0��ʾ��ǰ����p->Ek_1��ʾ�ϴε����
	EC = p->Ek_0 - p->Ek_1;//��ǰ����ȥ�ϴ����������0����ʾ����ڱ��	
	
//	p->DDD_NOW = p->a2 * EC *0.2f + p->DDD_LAST *0.8f; //����ȫ΢�֣�p->a2����΢��ϵ��Kd
//	p->DDD_LAST = p->DDD_NOW;	
	
	//���ַ���PID�������ͻ���
//	ABS_E = fabsf(p->Ek_0);
//	if(ABS_E < 100)//�����ǰ�����100���ڣ����������л���
//	{
//		III_temp = 1.0f;
//	}
//	else if(ABS_E <1000)//�����ǰ�����100-1100֮�ڣ������Ըı�����ۼƵ�����ϵ��1  -  0.01
//	{
//		III_temp = 1.0f - 0.00099f * (ABS_E - 100);
//	}	
//	else if(ABS_E <3000)//�����ǰ�����1100-3000֮�ڣ������ۻ���ϵ����Ϊ0.000001
//	{
//		III_temp = 0.000001f;
//	}		
//	else//�����ǰ������3000������ȫȡ����������
//	{
//		III_temp = 0.000f;
//	}	
//	
////	p->Ek_2        = p->Ek_2 + p->Ek_0*III_temp;
//if(ABS_E > 20)//�����ǰ�����20���ڣ����������л���
//	p->Ek_2        = p->Ek_2;
//else
	p->Ek_2        = p->Ek_2 + p->Ek_0 * Irate;
	
	if( p->Ek_2 > p->IntegroMax * 0.9f) p->Ek_2   =  p->IntegroMax * 0.9f;//�����޷�
	if( p->Ek_2 < p->IntegroMin * 0.9f) p->Ek_2   =  p->IntegroMin * 0.9f;		

	p->N_Output      = (            p->a0 * p->Ek_0 * 1 \
														+     p->a1 * p->Ek_2 \
														);          //PID����

//	p->N_Output      = (            p->a0 * p->Ek_0 \
//														+     p->a1 * p->Ek_2 );          //PID����	

	
	p->N_Output = p->N_Output * 0.3f + p->L_Output * 0.7f;//LPF

	if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;//PID����޷�
	if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

	p->Ek_1        = p->Ek_0;//������ʷֵ
	p->L_Output    = p->N_Output;

	return p->N_Output;
}

PID_DataType Loc_pid_calc_3( PID_TypeDef *p,PID_DataType Irate, PID_DataType ref, PID_DataType fb)
{
	static float EC;//�������ı仯��
	static float ABS_E;//�������ľ���ֵ
	static float III_temp;//���������Loc_pid_calc������ʹ�ã������ȫ�ֱ�����Ŀ���ǿ����ڷ����ʱ��ʵʱ�۲쵽��������ı仯������ں����ڲ����壬���ڷ���ʱ�����ڹ۲죩	
	
//	if(ref<50)
//		ref =50;
	
	p->Ek_0	= ref - fb; //�������������趨ֵ��ʵ�ʲ���ֵ��ƫ��

	//p->Ek_0��ʾ��ǰ����p->Ek_1��ʾ�ϴε����
	EC = p->Ek_0 - p->Ek_1;//��ǰ����ȥ�ϴ����������0����ʾ����ڱ��	
	
	p->DDD_NOW = p->a2 * EC * 300 *0.06f + p->DDD_LAST *0.94f; //����ȫ΢�֣�p->a2����΢��ϵ��Kd
	p->DDD_LAST = p->DDD_NOW;	
	
	//���ַ���PID�������ͻ���
//	ABS_E = fabsf(p->Ek_0);
//	if(ABS_E < 100)//�����ǰ�����100���ڣ����������л���
//	{
//		III_temp = 1.0f;
//	}
//	else if(ABS_E <1000)//�����ǰ�����100-1100֮�ڣ������Ըı�����ۼƵ�����ϵ��1  -  0.01
//	{
//		III_temp = 1.0f - 0.00099f * (ABS_E - 100);
//	}	
//	else if(ABS_E <3000)//�����ǰ�����1100-3000֮�ڣ������ۻ���ϵ����Ϊ0.000001
//	{
//		III_temp = 0.000001f;
//	}		
//	else//�����ǰ������3000������ȫȡ����������
//	{
//		III_temp = 0.000f;
//	}	
//	
////	p->Ek_2        = p->Ek_2 + p->Ek_0*III_temp;
//if(ABS_E > 20)//�����ǰ�����20���ڣ����������л���
//	p->Ek_2        = p->Ek_2;
//else
	p->Ek_2        = p->Ek_2 + p->Ek_0 * Irate;
	
	if( p->Ek_2 > p->IntegroMax * 1.0f) p->Ek_2   =  p->IntegroMax * 1.0f;//�����޷�
	if( p->Ek_2 < p->IntegroMin * 1.0f) p->Ek_2   =  p->IntegroMin * 1.0f;		


	p->N_Output      = (            p->a0 * p->Ek_0 * 1 \
														+     p->a1 * p->Ek_2 \
														+     p->DDD_NOW );          //PID����

//	p->N_Output      = (            p->a0 * p->Ek_0 * 0.01f \
//														+     p->a1 * p->Ek_2 \
//														);          //PID����

//	p->N_Output      = (            p->a0 * p->Ek_0 \
//														+     p->a1 * p->Ek_2 );          //PID����	

	
	p->N_Output = p->N_Output * 0.1f + p->L_Output * 0.9f;//LPF

	if(p->N_Output > p->OutMax)p->N_Output   =  p->OutMax;//PID����޷�
	if(p->N_Output < p->OutMin)p->N_Output   =  p->OutMin;

	p->Ek_1        = p->Ek_0;//������ʷֵ
	p->L_Output    = p->N_Output;

	return p->N_Output;
}


//λ��ʽPID������ݺ���
void Loc_pid_clc( PID_TypeDef   *p)
{
    p->Ek_0             = 0;     //  Error[k]
    p->Ek_1             = 0;     //  Error[k-1]
    p->Ek_2             = 0;     //  Error[k-2]
    p->L_Output     = 0;
    p->N_Output     = 0;
}


 float      Xn,Xn_1,Xn_2,Xn_3;   //��ַ�������������ѹ-�ο���ѹ
 float      Yn,Yn_1,Yn_2,Yn_3;   //��ַ����������Ŵ������Verror
 float kx0=136.152858f;            //��ַ���ϵ������Ӧ��ַ��̸���  OK��
 float kx1=-2.807968f;
 float kx2=-0.620162f;
 float kx3=112.212783f;
 float ky1=-0.811847f;
 float ky2=-0.179303f;
 float ky3=-0.008850f;
float  type3_clc(float vout_ref,float vout_temp)//TYPE3�㷨
{
	 Xn=(float)(vout_ref-vout_temp)*0.0002f;          //�ο���ѹ-�����ѹ
	 Yn=kx0*Xn+kx1*Xn_1+kx2*Xn_2+kx3*Xn_3-ky1*Yn_1-ky2*Yn_2-ky3*Yn_3; //������̴���ģ���·type3������

	 if(Yn>=1499){Yn=1499;}					     //���ռ�ձȹ�95%                          
	 else if(Yn<=0.00f){Yn=0.00f;}              //��Сռ�ձ�0.0%
	 
	 Xn_3=Xn_2;Xn_2=Xn_1;Xn_1=Xn;							 //��λ				 														 
	 Yn_3=Yn_2;Yn_2=Yn_1;Yn_1=Yn;	
	 return Yn;	 	 
}


 float        Kp_V=120.0f;             //����ϵ��
 float        Ki_V=0.3f;             //����ϵ��
 float        Kd_V=50.0f;             //΢��ϵ��
 float     	 error1_V=0.0f;         //��ǰ������趨ֵ���
 float      	 error2_V=0.0f;         //��ǰ������趨ֵ���
 float      	 e1_V=0.0f;             //�ϴ�������趨ֵ���
 float      	 e2_V=0.0f;             //���ϴ�������趨ֵ���
 

 __IO uint16_t Pulse_width=0;
float PID_BM_clc(float Vset_target,float Vout_actual)     //������PID
{ 
	static float Yn=0;
	//�����ѹPID��·
	error1_V=(Vset_target-Vout_actual);                       //��ǰ���
	e1_V=error1_V-error2_V;                                          //�������仯��
	Yn+=Kp_V*e1_V+Ki_V*error1_V+Kd_V*(e1_V-e2_V);      //�Ż�֮���PID�㷨(Kp*(ek-ek_1)+Ki*ek+Kd*(ek-2ek_1+ek_2))
	error2_V=error1_V;e2_V=e1_V;                                     //��λ
	
	//if(error1_V<-0.2f)ERROR_Increase_V-=0.1f;
	//�޷�
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



