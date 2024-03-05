#include "JointMove.h"

JOINTSTATUE state;
float Tt=0.3;
void JointMove(void const * pvParameters)
{
	while(1)
	{
//		if(state==POSITION)
//		{
//		//		if(id==2)
//					
//				  
//				  Stand_left1_p(-x_distance,y_distance_left);
//				  Stand_right1_p(x_distance,y_distance_right);	
//			 	  osDelay(1);
//				  Stand_left2_p(-x_distance,y_distance_left);
//				  Stand_right2_p(x_distance,y_distance_right);
//					osDelay(1);
//		}
//		if(state==TORQUE)
//		{
					//T_left1=Tt;T_right1=Tt;T_left2=-Tt;T_right2=-Tt;
					Stand_left1_t(T_left1);
					Stand_right1_t(T_right1);	
					osDelay(1);
					Stand_left2_t(T_left2);
					Stand_right2_t(T_right2);
					osDelay(1);	
					//correct_time++;
//		}
			
	}
}