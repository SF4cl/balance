#ifndef SETTING_H
#define SETTING_H

#include "struct_typedef.h"


#define LENGTH 																				1000

#define MOTOR_OFFLINE_TIMEMAX  												50
#define REMOTE_OFFLINE_TIMEMAX 												550
#define AIMBOT_OFFLINE_TIMEMAX  											550
#define REFEREE_OFFLINE_TIMEMAX 											3000

#define PARAMETER_FILE "Setting.h"



// imu安装方向
#define IMU_DIRECTION_yrxz_XYZ

// 电机ID分配
#define JOINT_COMMAND_ID															0x100
#define JOINT_SETTING_ID															0x105
#define JOINT_FEEDBACK_TORQUE_ID											0x1F0
#define JOINT_FEERBACK_LEFT1_ID												0x1F2
#define JOINT_FEERBACK_LEFT2_ID												0x1F1
#define JOINT_FEERBACK_RIGHT1_ID											0x1F3
#define JOINT_FEERBACK_RIGHT2_ID											0x1F4
#define DebugDataId																		0x3ff

#define YawMotorId                                   	0x205

#define FEET_MOTOR1_TRANSMIT_ID         			  		  0x281
#define FEET_MOTOR2_TRANSMIT_ID         			  		  0x282

#define FEET_MOTOR1_RECEIVE_ID         	  						0x201
#define FEET_MOTOR2_RECEIVE_ID         	 			 		    0x202


#define IMU_GYRO_YAW_BIAS    													-0.00465f

#define FALLOW_ANGLE              										85.5f
#define ROTING_SPEED																	15.0f

#define TORQUE_K																			2730.0f
#define TORQUE_W																			0.754f


#define FOLLOW_KP																			5.0f
#define FOLLOW_KI																			0.0f
#define FOLLOW_KD																			100.0f

#define TURN_KP																				4500.0f
#define TURN_KI																				0.0f
#define TURN_KD																				10000.0f

#define ROLL_KP																				0.3f
#define ROLL_KI																				0.001f
#define ROLL_KD																				0.001f



#define LEG_F_MAX																			280
#define LEG_I_MAX																			120


#define JOINT_RUN_MODE 																10
#define JOINT_RESET_MODE															0
#define JOINT_KP 																			0.1
#define JOINT_KW 																			0.01
#define JOINT_W 																			0
#define JOINT_T 																			0
#define JOINT_RIGHT1_ID 															0
#define JOINT_RIGHT2_ID 															1
#define JOINT_LEFT1_ID 																0
#define JOINT_LEFT2_ID 																2



#endif

