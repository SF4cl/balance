#include "CalculateThread.h"



					
Chassis_t Chassis;
RC_ctrl_t Remote;

EulerSystemMeasure_t    Imu;

void CalculateThread(void *argument)
{
		Calculate_Init();
		//osDelay(15000);
		while(1)
		{
				ChassisStateUpdate();
				ChassisModeUpdate();
				Calculate_LQR();
				osDelay(1);	
		}
}



