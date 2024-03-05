#include "AttitudeThread.h"

uint16_t whoami;
uint16_t pwmimu = 10;


fp32 INS_quat[4] = {0.707f, 0.7074f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.Å·À­½Ç µ¥Î» rad

int califlag=0,t=0;
float CalcountG[3],CalcountA[3];
void AttitudeThread(void *argument)
{
		osDelay(100);  
//		for(int i=0;i<100000&&Temperature<44.0f;i++)
//		{
//				whoami = ICM_42688_ReadReg(0x75);
//				ICM_42688_ReadAccel();
//		//		if(Accel[0]==0x00 && Accel[1]==0x00 && Accel[2]==0x00)
//		//			IMU42688_Init();
//		//		if(Accel[0]==-32768 && Accel[1]==-32768 && Accel[2]==-32768)
//		//			IMU42688_Init();		
//					
//				ICM_42688_ReadGyro();
//				ICM_42688_ReadTem();
//				
//				AccelCorrected[0] = ((float)Accel[0] + AccelCal[0]) * LSB_ACC_GYRO[0];
//				AccelCorrected[1] = ((float)Accel[1] + AccelCal[1]) * LSB_ACC_GYRO[0];
//				AccelCorrected[2] = ((float)Accel[2] + AccelCal[2]) * LSB_ACC_GYRO[0];
//				GyroCorrected[0] = ((float)Gyro[0] + GyroCal[0]) * LSB_ACC_GYRO[1];
//				GyroCorrected[1] = ((float)Gyro[1] + GyroCal[1]) * LSB_ACC_GYRO[1];
//				GyroCorrected[2] = ((float)Gyro[2] + GyroCal[2]) * LSB_ACC_GYRO[1];			
//				
//				pwmimu=50;/*
//				if(Temperature>45)	pwmimu=0;
//				else if(Temperature<40)	pwmimu=25;
//				else	pwmimu=25-4*(Temperature-40);
//				__HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, pwmimu);*/
//				//testX=FSqrt(INS_angle[1]);
//				//printf("%f,%f,%f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
//				osDelay(1);   				
//		}
		
//		for(int i=0;i<5000;i++)
//		{
//				whoami = ICM_42688_ReadReg(0x75);
//				ICM_42688_ReadAccel();
//		//		if(Accel[0]==0x00 && Accel[1]==0x00 && Accel[2]==0x00)
//		//			IMU42688_Init();
//		//		if(Accel[0]==-32768 && Accel[1]==-32768 && Accel[2]==-32768)
//		//			IMU42688_Init();		
//					
//				ICM_42688_ReadGyro();
//				ICM_42688_ReadTem();
//				
//				AccelCorrected[0] = ((float)Accel[0] + AccelCal[0]) * LSB_ACC_GYRO[0];
//				AccelCorrected[1] = ((float)Accel[1] + AccelCal[1]) * LSB_ACC_GYRO[0];
//				AccelCorrected[2] = ((float)Accel[2] + AccelCal[2]) * LSB_ACC_GYRO[0];
//				GyroCorrected[0] = ((float)Gyro[0] + GyroCal[0]) * LSB_ACC_GYRO[1];
//				GyroCorrected[1] = ((float)Gyro[1] + GyroCal[1]) * LSB_ACC_GYRO[1];
//				GyroCorrected[2] = ((float)Gyro[2] + GyroCal[2]) * LSB_ACC_GYRO[1];			
//				CalcountG[0]+=GyroCorrected[0];
//				CalcountG[1]+=GyroCorrected[1];
//				CalcountG[2]+=GyroCorrected[2];
//				CalcountA[0]+=GyroCorrected[0];
//				CalcountA[1]+=GyroCorrected[1];
//				CalcountA[2]+=GyroCorrected[2];			
//				
//				if(Temperature>45)	pwmimu=0;
//				else if(Temperature<40)	pwmimu=25;
//				else	pwmimu=25-4*(Temperature-40);
//				__HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, pwmimu);
//				//testX=FSqrt(INS_angle[1]);
//				//printf("%f,%f,%f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
//				osDelay(1);   				
//		}		
    while (1)
    {
				whoami = ICM_42688_ReadReg(0x75);
				ICM_42688_ReadAccel();
		//		if(Accel[0]==0x00 && Accel[1]==0x00 && Accel[2]==0x00)
		//			IMU42688_Init();
		//		if(Accel[0]==-32768 && Accel[1]==-32768 && Accel[2]==-32768)
		//			IMU42688_Init();		
					
				ICM_42688_ReadGyro();
				ICM_42688_ReadTem();
				

				AccelCorrected[0] = ((float)Accel[0] + AccelCal[0]) * LSB_ACC_GYRO[0];
				AccelCorrected[1] = ((float)Accel[1] + AccelCal[1]) * LSB_ACC_GYRO[0];
				AccelCorrected[2] = ((float)Accel[2] + AccelCal[2]) * LSB_ACC_GYRO[0];
				GyroCorrected[0] = ((float)Gyro[0] + GyroCal[0]) * LSB_ACC_GYRO[1] ;//- CalcountG[0]/5000.0f;
				GyroCorrected[1] = ((float)Gyro[1] + GyroCal[1]) * LSB_ACC_GYRO[1] ;//- CalcountG[1]/5000.0f;
				GyroCorrected[2] = ((float)Gyro[2] + GyroCal[2]) * LSB_ACC_GYRO[1] ;//- CalcountG[2]/5000.0f;			
				if(califlag)
				{
						CalcountG[0]=((float)Gyro[0]+GyroCal[0]+CalcountG[0]*(float)t)/(float)(t+1);
						CalcountG[1]=((float)Gyro[1]+GyroCal[1]+CalcountG[1]*(float)t)/(float)(t+1);					
						CalcountG[2]=((float)Gyro[2]+GyroCal[2]+CalcountG[2]*(float)t)/(float)(t+1);					
						t++;
				}
				else
						t=0;
				
				MahonyAHRSupdateIMU(INS_quat, -GyroCorrected[1], -GyroCorrected[2], GyroCorrected[0], AccelCorrected[1], AccelCorrected[2], -AccelCorrected[0]);
				get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
				if(Temperature>40)	pwmimu=0;
				else if(Temperature<35)	pwmimu=25;
				else	pwmimu=28-4*(Temperature-35);
				__HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, pwmimu);
				//testX=FSqrt(INS_angle[1]);
				//printf("%f,%f,%f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
				
				count_time[4]++;
				osDelay(1);   
		}
}

void AHRS_init(fp32 quat[4], fp32 accel[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

