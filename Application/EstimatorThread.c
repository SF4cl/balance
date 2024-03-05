 #include "EstimatorThread.h"
 
 #define Freq 0.001
 
 
float va[2];
float m[2];
float bias_a;


float x_estimator=0;
KalmanFilter_t Height_KF;
 void Estimator_Init(void)
 {
     static float P_Init[4] =
     {
         1, 0, 
         0, 10,  
     };
//     static float B_Init[2] =
//     {
//         0,1
//     };
		 static float F_Init[4] =
     {
         1, Freq, 
         0, 1,
     };
//     static float Q_Init[4] =
//     {
//         0.25*Freq*Freq*Freq*Freq, 0.5*Freq*Freq*Freq,
//         0.5*Freq*Freq*Freq,        Freq*Freq,  
//     };
			static float H_Init[4] =
			{
					1, 0,
				  0, 1
			};
			static float Q_Init[4] =
			{
					10, 0,
					0, 1
			};
			static float R_Init[4] = 
			{
					1, 0,
					0, 1
			};
		 
     // 设置最小方差
     static float state_min_variance[2] = {0.005, 0.1};
     
     // 开启自动调整
     Height_KF.UseAutoAdjustment = 0;
 
     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
     static uint8_t measurement_reference[2] = {1, 2};
 
     static float measurement_degree[2] = {1, 1};     
     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
//       |1   0   0|
//       |1   0   0|
//       |0   0   1|
 
     static float mat_R_diagonal_elements[2] = {30, 25};
     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
//       |30   0   0|
//       | 0  25   0|
//       | 0   0  35|
 
     Kalman_Filter_Init(&Height_KF, 2, 0, 2);
 
     // 设置矩阵值
     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
		 memcpy(Height_KF.R_data, R_Init, sizeof(R_Init));
		 memcpy(Height_KF.H_data, H_Init, sizeof(H_Init));
		 
//		 memcpy(Height_KF.B_data, B_Init, sizeof(B_Init));
     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 }
 

 // 测量数据更新应按照以下形式 即向MeasuredVector赋值
void Wheel_Update(float x)
{
    Height_KF.MeasuredVector[0] = x;//baro_height;
		m[0]=Height_KF.MeasuredVector[0];
		if(Height_KF.MeasuredVector[0]<0.05 && Height_KF.MeasuredVector[0]>-0.05)
				Height_KF.F_data[0] = 0.001;
		else
				Height_KF.F_data[0] = 10;
	
}
bool reset_flag = false;
void Acc_Update(float x)
{

		if(!reset_flag)
				x-=bias_a;
		Height_KF.MeasuredVector[1] = x;//acc.z;
		m[1]=Height_KF.MeasuredVector[1];

}
	
void Current_Update(float x)
{
		Height_KF.ControlVector[0]=x;
}


float v_imu = 0;
float x_imu = 0;

int cnt = 0;


void EstimatorThread(void const *pvParameters)
{
		Estimator_Init();
		osDelay(4000);
		while(1)
		{
				Kalman_Filter_Update(&Height_KF);
				x_estimator+=Height_KF.FilteredValue[0]*Freq;
				va[0]=Height_KF.FilteredValue[0];
				va[1]=Height_KF.FilteredValue[1];
				v_imu+=m[1]*Freq;
				x_imu+=v_imu*Freq;

				if(reset_flag)
				{
					bias_a=(m[1]+bias_a*cnt)/(cnt+1);
					cnt++;
				}
							
				osDelay(1);	
		}	
	
}