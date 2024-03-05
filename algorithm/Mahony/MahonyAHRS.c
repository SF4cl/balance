//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	1.0f	// 2 * proportional gain
#define twoKiDef	0.0f	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions


float FSqrt(float x)
{
		float x_in,y_out;
		x_in=x;
		arm_sqrt_f32(x_in,&y_out);
		return y_out;
}



void getRotation(float *Quaternion, float *rt_mat)
{
  rt_mat[0] = 1 - 2 * (Quaternion[2] * Quaternion[2]) - 2 * (Quaternion[3] * Quaternion[3]);
  rt_mat[1] = 2 * Quaternion[1] * Quaternion[2] - 2 * Quaternion[0] * Quaternion[3];
  rt_mat[2] = 2 * Quaternion[1] * Quaternion[3] + 2 * Quaternion[0] * Quaternion[2];
  rt_mat[3] = 2 * Quaternion[1] * Quaternion[2] + 2 * Quaternion[0] * Quaternion[3];
  rt_mat[4] = 1 - 2 * (Quaternion[1] * Quaternion[1]) - 2 * (Quaternion[3] * Quaternion[3]);
  rt_mat[5] = 2 * Quaternion[2] * Quaternion[3] - 2 * Quaternion[0] * Quaternion[1];
  rt_mat[6] = 2 * Quaternion[1] * Quaternion[3] - 2 * Quaternion[0] * Quaternion[2];
  rt_mat[7] = 2 * Quaternion[2] * Quaternion[3] + 2 * Quaternion[0] * Quaternion[1];
  rt_mat[8] = 1 - 2 * (Quaternion[1] * Quaternion[1]) - 2 * (Quaternion[2] * Quaternion[2]);
}
float g_t=0;
float a_q[3],a_g[3];
float mat[9]={0},Quat[4];
//uint32_t cou=0;
float halfex_w, halfey_w, halfez_w;
void Normalise(float a[3])
{
		float recipNorm = invSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		a[0] *= recipNorm;
		a[1] *= recipNorm;
		a[2] *= recipNorm;   		
}
float v_est=0;
//void MahonyAHRSupdateIMU_X(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
//	float recipNorm;
//	float halfvx, halfvy, halfvz;
//	float halfex = 0, halfey = 0, halfez = 0;
//	float qa, qb, qc;
//	float imua;
//	float g = 9.746f,th;
//	
//	
//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {       
//		g_t=(g_t*cou+FSqrt(ax * ax + ay * ay + az * az))/(cou+1);
//		cou++;
//		a_g[0] = ax;//FSqrt(ax * ax + ay * ay + az * az - g * g);
//		//if(ax<0)	a_g[0]*=-1;
//		a_g[1] = ay;
//		a_g[2] = az;
//		
//		//a_g[1]=FSqrt(a_g[0]*a_g[0]+a_g[1]*a_g[1]);
//		//a_g[0]=0;
//		
//		if(ax * ax + ay * ay + az * az < g*g)
//				th=asin(1);
//		else
//				th=asin(g/FSqrt(ax * ax + ay * ay + az * az));
//		Quat[0]=-q[0];
//		Quat[1]=q[1];
//		Quat[2]=q[2];
//		Quat[3]=q[3];
//		getRotation(Quat,mat);
//		// Estimated direction of gravity and vector perpendicular to magnetic flux
//		if(ax<0)
//		{
//				a_q[0]=(mat[0]*0+mat[1]*arm_cos_f32(th)+mat[2]*arm_sin_f32(th))/2.0;
//				a_q[1]=(mat[3]*0+mat[4]*arm_cos_f32(th)+mat[5]*arm_sin_f32(th))/2.0;		
//				a_q[2]=(mat[6]*0+mat[7]*arm_cos_f32(th)+mat[8]*arm_sin_f32(th))/2.0;		

////				a_g[1]*=-1;
//		}
//		else
//		{
//				a_q[0]=(mat[0]*0-mat[1]*arm_cos_f32(th)+mat[2]*arm_sin_f32(th))/2.0;
//				a_q[1]=(mat[3]*0-mat[4]*arm_cos_f32(th)+mat[5]*arm_sin_f32(th))/2.0;		
//				a_q[2]=(mat[6]*0-mat[7]*arm_cos_f32(th)+mat[8]*arm_sin_f32(th))/2.0;	
//			
//			
//				
//		}
//		v_est+=a_q[1]/sin(th)*g*0.001;
//		//a_q[0]=(q[1] * q[3] - q[0] * q[2]);
//		//a_q[1]=(q[0] * q[1] + q[2] * q[3]);	
//		//a_q[2]=(q[0] * q[0] - 0.5f + q[3] * q[3]);		
//		//a_q[0]=FSqrt(a_q[0]*a_q[0]+a_q[1]*a_q[1]);
//		//a_q[1]=0;
//		// Error is sum of cross product between estimated and measured direction of gravity
//		

////		if (a_q[2] > g + 0.5 || a_q[2] < g - 0.5)
////		{
////				halfex=0;
////				halfey=0;
////				halfez=0;
////		}
////		else
////		{
//				//Normalise(a_q);
//				Normalise(a_g);
//				halfex = -(a_q[1] * a_g[2] - a_q[2] * a_g[1]);	
//				halfey = -(a_q[2] * a_g[0] - a_q[0] * a_g[2]);
//				halfez = -(a_q[0] * a_g[1] - a_q[1] * a_g[0]);
////				halfex = mat[0]*halfex_w+mat[1]*halfey_w+mat[2]*halfez_w;			
////				halfey = mat[3]*halfex_w+mat[4]*halfey_w+mat[5]*halfez_w;			
////				halfez = mat[6]*halfex_w+mat[7]*halfey_w+mat[8]*halfez_w;			
////		}
////		halfex = (ay * halfvz - az * halfvy);
////		halfey = (az * halfvx - ax * halfvz);
////		halfez = (ax * halfvy - ay * halfvx);


//		// Compute and apply integral feedback if enabled
//		if(twoKi > 0.0f) {
//			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
//			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
//			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
//			gx += integralFBx;	// apply integral feedback
//			gy += integralFBy;
//			gz += integralFBz;
//		}
//		else {
//			integralFBx = 0.0f;	// prevent integral windup
//			integralFBy = 0.0f;
//			integralFBz = 0.0f;
//		}

//		// Apply proportional feedback
////		gx += twoKp * halfex;
////		gy += twoKp * halfey;
////		gz += twoKp * halfez;
//	}
//	
//	// Integrate rate of change of quaternion
//	gx *= ((1.0f / sampleFreq));		// pre-multiply common factors
//	gy *= ((1.0f / sampleFreq));
//	gz *= ((1.0f / sampleFreq));
//	qa = q[0];
//	qb = q[1];
//	qc = q[2];
//	q[0] += (-qb * gx - qc * gy - q[3] * gz);
//	q[1] += (qa * gx + qc * gz - q[3] * gy);
//	q[2] += (qa * gy - qb * gz + q[3] * gx);
//	q[3] += (qa * gz + qb * gy - qc * gx); 
////	halfex *= 0.5f * sampleFreq;
////	halfey *= 0.5f * sampleFreq;
////	halfez *= 0.5f * sampleFreq;
////	q[0] += (-qb * halfex - qc * halfey - q[3] * halfez);
////	q[1] += (qa * halfex + qc * halfez - q[3] * halfey);
////	q[2] += (qa * halfey - qb * halfez + q[3] * halfex);
////	q[3] += (qa * halfez + qb * halfey - qc * halfex); 
//	
//	// Normalise quaternion
//	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
//	q[0] *= recipNorm;
//	q[1] *= recipNorm;
//	q[2] *= recipNorm;
//	q[3] *= recipNorm;
//}

float updated_g[3];

void MahonyAHRSupdateIMU_X(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex = 0, halfey = 0, halfez = 0;
	float qa, qb, qc;

	//add the bias
//	gx -= -0.03042388f;
//	gy -= -0.09412107f;
//	gz -= -0.04485130f;
	
	updated_g[0] = gx;
	updated_g[1] = gy;
	updated_g[2] = gz;
	
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
//		gx += twoKp * halfex;
//		gy += twoKp * halfey;
//		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion

	halfex *= (1.0f / sampleFreq);
	halfey *= (1.0f / sampleFreq);
	halfez *= (1.0f / sampleFreq);
	q[0] += (-qb * halfex - qc * halfey - q[3] * halfez);
	q[1] += (qa * halfex + qc * halfez - q[3] * halfey);
	q[2] += (qa * halfey - qb * halfez + q[3] * halfex);
	q[3] += (qa * halfez + qb * halfey - qc * halfex); 
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}


//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
		
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
//		gx += twoKp * halfex;
//		gy += twoKp * halfey;
//		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex = 0, halfey = 0, halfez = 0;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
//	halfex *= (0.5f * (1.0f / sampleFreq));
//	halfey *= (0.5f * (1.0f / sampleFreq));
//	halfez *= (0.5f * (1.0f / sampleFreq));
//	q[0] += (-qb * halfex - qc * halfey - q[3] * halfez);
//	q[1] += (qa * halfex + qc * halfez - q[3] * halfey);
//	q[2] += (qa * halfey - qb * halfez + q[3] * halfex);
//	q[3] += (qa * halfez + qb * halfey - qc * halfex); 
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}



//====================================================================================================
// END OF CODE
//====================================================================================================
