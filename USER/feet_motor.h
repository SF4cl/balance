#ifndef FEET_MOTOR_H
#define FEET_MOTOR_H

#include "fdcan.h"
#include "Setting.h"


typedef struct{
    uint16_t    ecd;
    int16_t     speed_rpm;
    int16_t     given_current;
    uint8_t     temperature;
    int16_t     last_ecd;
		int16_t			last_speed_rpm;
}feet_motor_measure_t;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
				(ptr)->last_speed_rpm = (ptr)->speed_rpm;      \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperature = (data)[6];                                   \
    }

		
		
extern feet_motor_measure_t LeftFootMotorMeasure;
extern feet_motor_measure_t RightFootMotorMeasure;		
		

extern void FEET_CONTROL(int16_t FEET_MOTOR_LEFT, int16_t FEET_MOTOR_RIGHT);

		
#endif
		