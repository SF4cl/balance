#ifndef __AttitudeThread_H
#define __AttitudeThread_H
#include "struct_typedef.h"
#include "MahonyAHRS.h"
#include "YC_ICM42688.h"
#include "cmsis_os.h"
#include "tim.h"
#include "stm32h7xx_it.h"


extern void AttitudeThread(void *argument);
extern void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);


extern fp32 INS_quat[4];
extern fp32 INS_angle[3];

#endif
