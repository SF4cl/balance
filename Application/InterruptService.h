#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include "main.h"
#include "struct_typedef.h"
#include "joint_motor.h"
#include "feet_motor.h"
#include "CanPacket.h"
#include "SendRecv.h"
#include  "control.h"
#include "CalculateThread.h"

extern EulerSystemMeasure_t    Imu;

void TimerTaskLoop1000Hz();
void TimerTaskLoop1000Hz();
void TimerTaskLoop500Hz_1();
void TimerTaskLoop500Hz_2();
void TimerTaskLoop100Hz();


#endif