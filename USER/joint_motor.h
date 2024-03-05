#ifndef JOINT_MOTOR_H
#define JOINT_MOTOR_H

#include "usart.h"
#include "SendRecv.h"
#include "movement_calc.h"
#include "Setting.h"
#include "main.h"


#define GATHER_TIME          1000;
#define JUMP_TIME            40;

#define RECEIVE_SIZE					78
#define SEND_SIZE							34


extern MOTOR_send motor_left1,motor_left2,motor_right1,motor_right2;
extern MOTOR_recv motor_left1_r,motor_left2_r,motor_right1_r,motor_right2_r;
extern double x_distance,y_distance_left,y_distance_right,angle;
extern void joint_motor_set_mode(int MODE);
extern void joint_send(uint32_t MotorID,MOTOR_send motor);
extern void Stand_left1_p(double x,double y);
extern void Stand_right1_p(double x,double y);
extern void Stand_left2_p(double x,double y);
extern void Stand_right2_p(double x,double y);
extern void Stand_left1_t(double T);
extern void Stand_right1_t(double T);
extern void Stand_left2_t(double T);
extern void Stand_right2_t(double T);
extern void fly();

typedef enum{
	gather_strength,
	jump,
	back
}fly_sate;


typedef struct
{
	uint8_t RX_flag:1;//IDLE receive flag

	uint16_t RX_Size;//receive length

	uint8_t RX_pData[RECEIVE_SIZE];//DMA receive buffer

	uint8_t TX_pData[SEND_SIZE];
}USART_SENDRECEIVETYPE;

extern USART_SENDRECEIVETYPE Usart2Type;
extern USART_SENDRECEIVETYPE Usart3Type;
extern uint8_t rx_buffer[50];


#endif

