#ifndef __IMUEKF_H__
#define __IMUEKF_H__

#include "struct_typedef.h"
#include "CalculateThread.h"
#include "arm_math.h"
#include "MahonyAHRS.h"
#include "kalman filter.h"

extern void EKFupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);


#endif
