#include "joint_motor.h"



USART_SENDRECEIVETYPE Usart2Type __attribute__((section(".ARM.__at_0x24000000")));
USART_SENDRECEIVETYPE Usart3Type __attribute__((section(".ARM.__at_0x24000000")));
uint8_t rx_buffer[50] __attribute__((section(".ARM.__at_0x24000000")));

//extern CAN_HandleTypeDef hcan2;
MOTOR_send motor_left1;
MOTOR_send motor_left2;
MOTOR_send motor_right1;
MOTOR_send motor_right2;
MOTOR_recv motor_left1_r;
MOTOR_recv motor_left2_r;
MOTOR_recv motor_right1_r;
MOTOR_recv motor_right2_r;
uint8_t fly_mode;
//static uint32_t             send_mail_box;
//static CAN_TxHeaderTypeDef  can_tx_message;
//static uint8_t              MotorSendBuffer[16];
//static uint8_t				JointSendBuffer[8];
double x_distance=0,y_distance_left=220,y_distance_right=220,angle=0;

// Can_Recv can_data;
//void get_joint_motor_measure(MOTOR_recv *motor,uint8_t* rx_data)                                    
//{ 	 
//	
//	 memcpy(&can_data,rx_data,12);
//     motor->Pos=can_data.Pos;
//	 motor->T=can_data.T;
//	 motor->W=can_data.W;
//}

//void joint_send(uint32_t MotorID,MOTOR_send motor)
//{
//	can_data.Pos=motor.Pos;
//	can_data.T=motor.T;
//	can_data.W=motor.W;
//	memcpy(JointSendBuffer,&can_data,8);
//    can_tx_message.IDE = CAN_ID_STD;
//    can_tx_message.RTR = CAN_RTR_DATA;
//    can_tx_message.DLC = 0x08;
//    can_tx_message.StdId = MotorID;
//    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, JointSendBuffer, &send_mail_box);
//}



void joint_motor_set_mode(int MODE)
{	
	
    motor_left1.mode = MODE;
		motor_left1.id=JOINT_LEFT1_ID;
		motor_left1_r.motor_id=JOINT_LEFT1_ID;
	
    motor_left2.mode = MODE;
		motor_left2.id=JOINT_LEFT2_ID;
		motor_left2_r.motor_id=JOINT_LEFT2_ID;
	
    motor_right1.mode = MODE;
		motor_right1.id=JOINT_RIGHT1_ID;
		motor_right1_r.motor_id=JOINT_RIGHT1_ID;
	
    motor_right2.mode = MODE;
		motor_right2.id=JOINT_RIGHT2_ID;
		motor_right2_r.motor_id=JOINT_RIGHT2_ID;
}

//void Stand_left1_p(double x,double y)
//{
//	//motor_left1.Pos=6;
//	motor_left1.K_P=JOINT_KP;
//	motor_left1.K_W=JOINT_KW;
//	motor_left1.T=JOINT_T;
//	motor_left1.W=JOINT_W;
//	motor_left1.Pos=Origin_angle_LEFT1-(float)calc_left(x,y)*9.1;
//	modify_data(&motor_left1);
//	HAL_UART_Transmit_DMA(&huart6,(uint8_t*)&(motor_left1.motor_send_data),34);                                                                                              
//}

//void Stand_left2_p(double x,double y)
//{
//	//motor_left2.Pos=3;
//	motor_left2.id=JOINT_LEFT2_ID;
//	motor_left2.K_P=JOINT_KP;
//	motor_left2.K_W=JOINT_KW;
//	motor_left2.T=JOINT_T;
//	motor_left2.W=JOINT_W;
//	motor_left2.Pos=Origin_angle_LEFT2+(float)calc_right(x,y)*9.1;
//	modify_data(&motor_left2);
//	HAL_UART_Transmit_DMA(&huart6,(uint8_t*)&(motor_left2.motor_send_data),34);
//                                                                                                
//}

//void Stand_right1_p(double x,double y)
//{
//  // motor_right1.Pos=2;
//		motor_right1.id=JOINT_RIGHT1_ID;
//	motor_right1.K_P=JOINT_KP;
//	motor_right1.K_W=JOINT_KW;
//	motor_right1.T=JOINT_T;
//	motor_right1.W=JOINT_W;
//	motor_right1.Pos=Origin_angle_RIGHT1-(float)calc_left(x,y)*9.1;
//	modify_data(&motor_right1);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)&(motor_right1.motor_send_data),34);
//                                                                                                
//}

//void Stand_right2_p(double x,double y)
//{
//    //motor_right2.Pos=2;
//	motor_right2.id=JOINT_RIGHT2_ID;
//	motor_right2.K_P=JOINT_KP;
//	motor_right2.K_W=JOINT_KW;
//	motor_right2.T=JOINT_T;
//	motor_right2.W=JOINT_W;
//	motor_right2.Pos=Origin_angle_RIGHT2+(float)calc_right(x,y)*9.1;
//	modify_data(&motor_right2);
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)&(motor_right2.motor_send_data),34);
//                 	
//}

void Stand_left1_t(double T)
{
	//motor_left1.Pos=6;
	motor_left1.K_P=0;
	motor_left1.K_W=0;
	motor_left1.Pos=0;
	motor_left1.T=T;
	modify_data(&motor_left1);
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);
	memcpy(Usart2Type.TX_pData,(uint8_t*)&(motor_left1.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);	
	HAL_UART_Transmit_DMA(&huart2,Usart2Type.TX_pData,SEND_SIZE);      
	//HAL_UART_Transmit(&huart2,(uint8_t*)&(motor_left1.motor_send_data),34,0xFF);       

}

void Stand_left2_t(double T)
{
	//motor_left2.Pos=3;
	motor_left2.K_P=0;
	motor_left2.K_W=0;
	motor_left2.Pos=0;
	motor_left2.T=T;
	modify_data(&motor_left2);
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);
	memcpy(Usart2Type.TX_pData,(uint8_t*)&(motor_left2.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);
	HAL_UART_Transmit_DMA(&huart2,Usart2Type.TX_pData,SEND_SIZE);    
	//HAL_UART_Transmit(&huart2,(uint8_t*)&(motor_left2.motor_send_data),34,0xFF);                                                                                                    
}

void Stand_right1_t(double T)
{
  // motor_right1.Pos=2;
	motor_right1.K_P=0;
	motor_right1.K_W=0;
	motor_right1.Pos=0;
	motor_right1.T=T;
	modify_data(&motor_right1);
//	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	memcpy(Usart3Type.TX_pData,(uint8_t*)&(motor_right1.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	HAL_UART_Transmit_DMA(&huart3,Usart3Type.TX_pData,SEND_SIZE);   
	//HAL_UART_Transmit(&huart3,(uint8_t*)&(motor_right1.motor_send_data),34,0xFF);                                                                                             
}

void Stand_right2_t(double T)
{
    //motor_right2.Pos=2;
	motor_right2.K_P=0;
	motor_right2.K_W=0;
	motor_right2.Pos=0;
	motor_right2.T=T;
	modify_data(&motor_right2);
//	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	memcpy(Usart3Type.TX_pData,(uint8_t*)&(motor_right2.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	HAL_UART_Transmit_DMA(&huart3,Usart3Type.TX_pData,SEND_SIZE); 
	//HAL_UART_Transmit(&huart3,(uint8_t*)&(motor_right2.motor_send_data),34,0xFF);                  	
}

//uint8_t fly_mode;
//uint16_t fly_counter;

//void fly()
//{
//	if(fly_mode == back)
//	{
//		joint_motor_set_mode(JOINT_RUN_MODE);
//		fly_mode = gather_strength;
//		fly_counter = GATHER_TIME;
//	}
//	else if(fly_mode == gather_strength)
//	{
//		if(fly_counter > 0)
//		{
//			fly_counter--;
//			y_distance_left = 150;
//			y_distance_right = 150;
//		}
//		else
//		{	
//			fly_mode = jump;
//			fly_counter = JUMP_TIME;
//			
//		}
//			
//	}
//	else if(fly_mode == jump)
//	{
//		if(fly_counter > 0)
//		{
//			fly_counter--;
//			y_distance_left = 300;
//			y_distance_right = 300;
//		}
//		else 
//		{			
//			joint_motor_set_mode(JOINT_RESET_MODE);
//			fly_counter--;
//			if((int)fly_counter<-800)
//				fly_mode = back;
//			
//		}
//	
//	}

//}



