#ifndef __JOINTMOVE_H__
#define __JOINTMOVE_H__

#include "Setting.h"
#include "joint_motor.h"
#include "cmsis_os.h"
#include "feet_motor.h"
#include "control.h"

typedef enum{
	POSITION,
	TORQUE
}JOINTSTATUE;


void JointMove(void const * pvParameters);

extern JOINTSTATUE state;

#endif