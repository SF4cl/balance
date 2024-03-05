#include "InterruptService.h"
#include "AttitudeThread.h"

#include <string.h>
#include <stdio.h>

#define printf(...)  HAL_UART_Transmit_DMA(&huart1,\
																				(uint8_t  *)u1_buf,\
																				sprintf((char*)u1_buf,__VA_ARGS__))
uint8_t u1_buf[55];
																				
																				
																				
fp32 Fabs(fp32 x)
{	
		if(x>0)
				return x;
		return -x;
}
fp32 Calc_x(fp32 x,fp32 dx)
{
		//if(Fabs(x+dx)<0.5f)
				return x+dx;
		return x;
}


uint32_t l_feet1=0,l_feet2=0;																				
float Xa,Xb,temp=0.000079f;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;

int t=0,t_max1=0,t_min1=10000,t_max2=0,t_min2=10000,t_max=0,t_min=10000;
CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];
int16_t t_r[4];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		float pos,w;
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
		
		if(hcan->Instance==CAN2)
		{
				switch (rx_header.StdId)
				{

//							case JOINT_FEERBACK_LEFT1_ID:
//							{
//									
//									memcpy(&pos,rx_data,4);
//									memcpy(&w,rx_data+4,4);									
//									u4_left=Calc_pos(pos,LEFT1);
//									du4_left=-w/9.1f;
//									break;
//							}
//							case JOINT_FEERBACK_LEFT2_ID:
//							{
//									
//									memcpy(&pos,rx_data,4);
//									memcpy(&w,rx_data+4,4);														
//									u1_left=Calc_pos(pos,LEFT2);
//									du1_left=-w/9.1f;
//									break;
//							}							
//							case JOINT_FEERBACK_RIGHT1_ID:
//							{
//									memcpy(&pos,rx_data,4);
//									memcpy(&w,rx_data+4,4);														
//									u4_right=Calc_pos(pos,RIGHT1);
//									du4_right=w/9.1f;
//									break;
//							}
//							case JOINT_FEERBACK_RIGHT2_ID:
//							{
//									
//									memcpy(&pos,rx_data,4);
//									memcpy(&w,rx_data+4,4);														
//									u1_right=Calc_pos(pos,RIGHT2);
//									du1_right=w/9.1f;
//									break;
//							}		
//							case JOINT_FEEDBACK_TORQUE_ID:
//							{
//									//correct_time++;
//									memcpy(t_r,rx_data,8);
//									T_left1_r=t_r[0]/1000.0f;
//									T_left2_r=t_r[1]/1000.0f;
//									T_right1_r=t_r[2]/1000.0f;
//									T_right2_r=t_r[3]/1000.0f;
//									
//									break;
//							}
							case FEET_MOTOR1_RECEIVE_ID:
							{
									correct_time1++;
									MotorProcess(rx_header.StdId, hcan, rx_data);
									dx_right=-((float)RightFootMotorMeasure.speed_rpm/19.2f/60.0f)*R*3.14159f*2.0f/*+l0_right*da_right*arm_cos_f32(a_right)+da_right*arm_sin_f32(a_right)*/-db*R;
								
									//if(Fabs(Chassis.vx)<0.01f)
											x_right=Calc_x(x_right,dx_right/1000.0f);
									l_feet1=SystemTimer;
									break;
							}
							case FEET_MOTOR2_RECEIVE_ID:
							{
									correct_time2++;
									MotorProcess(rx_header.StdId, hcan, rx_data);
									dx_left=((float)LeftFootMotorMeasure.speed_rpm/19.2f/60.0f)*R*3.14159f*2.0f/*+l0_left*da_left*arm_cos_f32(a_left)+da_left*arm_sin_f32(a_left)*/-db*R;
									//if(Fabs(Chassis.vx)<0.01f)
											x_left=Calc_x(x_left,dx_left/1000.0f);
									l_feet2=SystemTimer;
									break;
							}
//							case DebugDataId:
//							{
//									memcpy(&(Xa),rx_data,4);
//									memcpy(&(Xb),rx_data+4,4);
//									printf("%f,%f",Xa,Xb);
//									break;
//							}
							default:
							{
									break;
							}
				}
		}
	else if(hcan->Instance==CAN1)
	{
			switch(rx_header.StdId)
			{
	

					case YawMotorId:
					{
							MotorProcess(rx_header.StdId,hcan,rx_data);
							break;
					}
					/*-------------------------------------云台数据下发接收-------------------------------------*/
					case DefaultAimStatusAndTargetId:
					{
							memcpy(&Aim,rx_data,sizeof(Aim_t));
							break;
					}
					case SentryAimStatusAndTargetId:
					{
							break;//哨兵相关，暂时不管
					}
					case DefaulPTZRequestAndStatusId:
					{
							memcpy(&PTZ,rx_data,sizeof(PTZ_t));
							//correct_time++;
							break;
					}
					case SentryPTZRequestAndStatusId:
					{
							break;//哨兵相关，不管
					}
					default:
					{
							break;
					}
		
		
		}
	
	
	
	}
	
}





void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop1000Hz();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop500Hz_1();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
    TimerTaskLoop500Hz_2();
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    TimerTaskLoop100Hz();
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}



void TimerTaskLoop1000Hz()
{
		GimbalEulerSystemMeasureUpdate(&Imu);	
		t++;
		if(t>=1000)
		{
				t=0;
				ttt1=correct_time1;
				if(correct_time1>t_max1)
						t_max1=correct_time1;
				if(correct_time1<t_min1)
						t_min1=correct_time1;
				correct_time1=0;
				ttt2=correct_time2;
				if(correct_time2>t_max2)
						t_max2=correct_time2;
				if(correct_time2<t_min2)
						t_min2=correct_time2;
				correct_time2=0;				
								correct_time1=0;
				ttt=correct_time;
				if(correct_time>t_max)
						t_max=correct_time;
				if(correct_time<t_min)
						t_min=correct_time;
				correct_time=0;				
		}
		
		
	



}

void TimerTaskLoop500Hz_1()
{
	




}

void TimerTaskLoop500Hz_2()
{
			
		
	
	
}

void TimerTaskLoop100Hz()
{
		//printf("%f,%f,%f,%f,%f,%f\n",INS_angle[0],INS_angle_x[0],INS_angle[1],INS_angle_x[1],INS_angle[2],INS_angle_x[2]);
}


//void USART6_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART1_IRQn 0 */

//  /* USER CODE END USART1_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
//  /* USER CODE BEGIN USART1_IRQn 1 */

//  /* USER CODE END USART1_IRQn 1 */
//}