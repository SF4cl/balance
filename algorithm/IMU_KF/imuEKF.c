#include "imuEKF.h"

#define sampleFreq	2000.0f	

KalmanFilter_t g_KF;
KalmanFilter_t e_KF;
float gVec[3];
float eVec[3];
bool g_KF_initialized = false;
bool e_KF_initialized = false;

void EKFinit(const float ax, const float ay, const float az)
{
		 float Status_Init[3] = {ax, ay, az};
		
		 float P_Init[9] =
		 {
				 1, 0.1, 0.1,
				 0.1, 1, 0.1,
		  	 0.1, 0.1, 1 
		 };
//     static float B_Init[2] =
//     {
//         0,1
//     };
     float F_Init[9] =
     {
         1, 0, 0,
         0, 1, 0,
         0, 0, 1
     };
//     static float Q_Init[4] =
//     {
//         0.25*Freq*Freq*Freq*Freq, 0.5*Freq*Freq*Freq,
//         0.5*Freq*Freq*Freq,        Freq*Freq,  
//     };
		 	float H_Init[9] =
			{
					1, 0, 0,
				  0, 1, 0,
					0, 0, 1
			};
			float Q_Init[9] =
			{
				1, 0, 0,
        0, 1, 0,
        0, 0, 1
			};
			float R_Init[9] = 
			{
				100000, 0, 0,
        0, 100000, 0,
        0, 0, 100000
			};
		 
     // 设置最小方差
     float state_min_variance[3] = {0.05, 0.05, 0.05};

      // 开启自动调整
     g_KF.UseAutoAdjustment = 1;
   
     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
     uint8_t measurement_reference[3] = {1, 2, 3};
 
     float measurement_degree[3] = {1, 1, 1};   

		 float mat_R_diagonal_elements[3] = {10000, 10000, 10000};
		 
     Kalman_Filter_Init(&g_KF, 3, 0, 3);
 
     // 设置矩阵值
		 memcpy(g_KF.xhatminus_data, Status_Init, sizeof(Status_Init));
		 memcpy(g_KF.xhat_data, Status_Init, sizeof(Status_Init));
     memcpy(g_KF.P_data, P_Init, sizeof(P_Init));
     memcpy(g_KF.F_data, F_Init, sizeof(F_Init));
     memcpy(g_KF.Q_data, Q_Init, sizeof(Q_Init));
		 memcpy(g_KF.R_data, R_Init, sizeof(R_Init));
		 memcpy(g_KF.H_data, H_Init, sizeof(H_Init));
//		 memcpy(Height_KF.B_data, B_Init, sizeof(B_Init));
     memcpy(g_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
     memcpy(g_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
     memcpy(g_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
     memcpy(g_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
}

float gvec[3];
void EKFupdate(float gx, float gy, float gz, float ax, float ay, float az)
{	
	
		// 空间换时间 避免重复运算
    float gxdt, gydt, gzdt;
    gxdt = gx * (1.0f/sampleFreq);
    gydt = gy * (1.0f/sampleFreq);
    gzdt = gz * (1.0f/sampleFreq);

    // 由于本例中状态转移矩阵为时变矩阵
    // 需要在卡尔曼滤波器更新前更新转移矩阵F的值
    g_KF.F_data[1] = gzdt;
    g_KF.F_data[2] = -gydt;

    g_KF.F_data[3] = -gzdt;
    g_KF.F_data[5] = gxdt;

    g_KF.F_data[6] = gydt;
    g_KF.F_data[7] = -gxdt;

    // 卡尔曼滤波器测量值更新
    // 不一定写在滤波器更新函数之前，也可写在与传感器通信的回调函数中
    g_KF.MeasuredVector[0] = ax;
    g_KF.MeasuredVector[1] = ay;
    g_KF.MeasuredVector[2] = az;

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(&g_KF);

    // 提取估计值
    for (uint8_t i = 0; i < 3; i++)
    {
        gVec[i] = g_KF.FilteredValue[i];
				gvec[i] = g_KF.FilteredValue[i];
    }

}

void KFinit()
{
		
		 float P_Init[9] =
		 {
				 1, 0.1, 0.1,
				 0.1, 1, 0.1,
		  	 0.1, 0.1, 1 
		 };
//     static float B_Init[2] =
//     {
//         0,1
//     };
     float F_Init[9] =
     {
         1.0f, 0, 0,
         0, 1.0f, 0,
         0, 0, 1.0f
     };
//     static float Q_Init[4] =
//     {
//         0.25*Freq*Freq*Freq*Freq, 0.5*Freq*Freq*Freq,
//         0.5*Freq*Freq*Freq,        Freq*Freq,  
//     };
		 	float H_Init[9] =
			{
					1, 0, 0,
				  0, 1, 0,
					0, 0, 1
			};
			float Q_Init[9] =
			{
				1, 0, 0,
        0, 1, 0,
        0, 0, 1
			};
			float R_Init[9] = 
			{
				100000, 0, 0,
        0, 100000, 0,
        0, 0, 100000
			};
		 
     // 设置最小方差
     float state_min_variance[3] = {0.05, 0.05, 0.05};
		 
		 // 开启自动调整
     g_KF.UseAutoAdjustment = 1;
   
     // 
     uint8_t measurement_reference[3] = {1, 2, 3};
 
     float measurement_degree[3] = {1, 1, 1};   

		 float mat_R_diagonal_elements[3] = {100000, 100000, 100000};

     Kalman_Filter_Init(&e_KF, 3, 0, 3);
 
     // 设置矩阵值
     memcpy(e_KF.P_data, P_Init, sizeof(P_Init));
     memcpy(e_KF.F_data, F_Init, sizeof(F_Init));
     memcpy(e_KF.Q_data, Q_Init, sizeof(Q_Init));
		 memcpy(e_KF.R_data, R_Init, sizeof(R_Init));
		 memcpy(e_KF.H_data, H_Init, sizeof(H_Init));
//		 memcpy(Height_KF.B_data, B_Init, sizeof(B_Init));
		 memcpy(e_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
     memcpy(e_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
     memcpy(e_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
     memcpy(e_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
}

void KFupdate(float gx, float gy, float gz, float halfvx, float halfvy, float halfvz)
{
//		static float Q_Init[9] =
//		{
//				1, 0, 0,
//        0, 1, 0,
//        0, 0, 1				
//		};
//		static float Static_Q_Init[9] =
//		{
//			  0.00001, 0, 0,
//        0, 0.00001, 0,
//        0, 0, 0.00001		
//		};
//		if (gx < 0.1 && gx > -0.1 && gy < 0.05 && gy > -0.1 && gz < 0.1 && gz > -0.1)
//		{
//				memcpy(e_KF.Q_data, Static_Q_Init, sizeof(Static_Q_Init));
//		}
//		else
//		{
//				memcpy(e_KF.Q_data, Static_Q_Init, sizeof(Static_Q_Init));
//		}
//	
		e_KF.xhat_data[0] = gx * (1.0f/sampleFreq);
		e_KF.xhat_data[1] = gy * (1.0f/sampleFreq);
		e_KF.xhat_data[2] = gz * (1.0f/sampleFreq);
		
		e_KF.MeasuredVector[0] = halfvx;
    e_KF.MeasuredVector[1] = halfvy;
    e_KF.MeasuredVector[2] = halfvz;


	
    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(&e_KF);

	  for (uint8_t i = 0; i < 3; i++)
    {
        eVec[i] = e_KF.FilteredValue[i];
    }
	
}

void EKFupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
{
		if (!g_KF_initialized)
		{
				EKFinit(ax, ay, az);
				g_KF_initialized = true;
				return;
		}
	
		EKFupdate(gx, gy, gz, ax, ay, az);
		
		float halfvx = (q[1] * q[3] - q[0] * q[2]);
		float halfvy = (q[0] * q[1] + q[2] * q[3]);
		float halfvz = (q[0] * q[0] - 0.5f + q[3] * q[3]);
		Normalise(gVec);

		// Error is sum of cross product between estimated and measured direction of gravity
		float halfex = (gVec[1] * halfvz  - gVec[2] * halfvy);
		float halfey = (gVec[2] * halfvx  - gVec[0] * halfvz);
		float halfez = (gVec[0] * halfvy  - gVec[1] * halfvx);
		
		if (!e_KF_initialized)
		{
				KFinit();
				e_KF_initialized = true;
				return;
		}
		KFupdate(gx, gy, gz, halfex, halfey, halfez);
		
		float qa = q[0];
		float qb = q[1];
		float qc = q[2];
		
//			halfex *= 0.5;
//			halfey *= 0.5;
//		  halfez *= 0.5;
		halfex = eVec[0] * 0.5f;
		halfey = eVec[1] * 0.5f;
		halfez = eVec[2] * 0.5f;
//		q[0] += (-qb * eVec[0] - qc * eVec[1] - q[3] * eVec[2]);
//		q[1] += (qa * eVec[0] + qc * eVec[2] - q[3] * eVec[1]);
//		q[2] += (qa * eVec[1] - qb * eVec[2] + q[3] * eVec[0]);
//		q[3] += (qa * eVec[2] + qb * eVec[1] - qc * eVec[0]);
		q[0] += (-qb * halfex - qc * halfey - q[3] * halfez);
		q[1] += (qa * halfex + qc * halfez - q[3] * halfey);
		q[2] += (qa * halfey - qb * halfez + q[3] * halfex);
		q[3] += (qa * halfez + qb * halfey - qc * halfex); 		
		
//	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
//	gy *= (0.5f * (1.0f / sampleFreq));
//	gz *= (0.5f * (1.0f / sampleFreq));
//	q[0] += (-qb * gx - qc * gy - q[3] * gz);
//	q[1] += (qa * gx + qc * gz - q[3] * gy);
//	q[2] += (qa * gy - qb * gz + q[3] * gx);
//	q[3] += (qa * gz + qb * gy - qc * gx); 
		
		// Normalise quaternion
		float recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] *= recipNorm;
		q[1] *= recipNorm;
		q[2] *= recipNorm;
		q[3] *= recipNorm;
		
}

