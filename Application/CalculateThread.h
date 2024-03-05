#ifndef __CalculateThread_H
#define __CalculateThread_H

#include "struct_typedef.h"
#include "feet_motor.h"
#include "control.h"
#include "user_lib.h"



typedef enum
{
	NOFORCE,
	STOP,
	FALLOW,
	ROTING,
	HIGHSPEED,
	JUMP
}ChassisMode_e;


typedef struct
{

	ChassisMode_e Mode;
	
	float vx;
	float vy;
	float wz;
	pid_type_def XYPid[4];
	pid_type_def WZPid;
	float Current[4];
	float WheelSpeed[4];
}Chassis_t;

extern Chassis_t Chassis;
extern EulerSystemMeasure_t    Imu;
extern void CalculateThread(void *argument);





#endif



