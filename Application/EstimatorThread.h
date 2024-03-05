#ifndef __ESTIMATORTHREAD_H__
#define __ESTIMATORTHREAD_H__

#include "struct_typedef.h"
#include "CalculateThread.h"
#include "arm_math.h"
#include "kalman filter.h"
//#include "user_lib.h"



extern void EstimatorThread(void const *pvParameters);


extern void Wheel_Update(float x);
extern void Acc_Update(float x);
extern void Current_Update(float x);


extern float x_estimator;

#endif