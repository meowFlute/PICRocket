/* 
 * File:   common.h
 * Author: Scott
 *
 * Created on May 7, 2015, 10:55 AM
 */

#ifndef COMMON_H
#define	COMMON_H
#define dt 0.01
#include <p24fv32ka301.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
//offset and scalars for servos
#define servo0_Offest -12.0   //verified to be pretty close Wednesday 6/3/2015
#define servo0_Scalar 1.2     //seems pretty spot on to both sides 6/6/2015
    
#define servo1_Offest 14.0    //verified "" 6/6/2015
#define servo1_Scalar 1.16    //seems close 6/6/2015    
    
#define servo2_Offest 14.0    //verified "" 6/6/2015
#define servo2_Scalar 1.0

#define servo3_Offest -5.0    //verified "" 6/6/2015
#define servo3_Scalar 1.0     
    
#define MAX_ROLL    15
#define MAX_PITCH   20
    
extern unsigned int OC4R, _TMR3, _PR3;
extern volatile unsigned int* PWMReg[]; 
    
extern float rollKD;
extern float rollKP;
extern float pitchKD;
extern float pitchKP;
extern float yawKD;
extern float yawKP;

extern float rollDesired;
extern float pitchDesired;
extern float yawDesired;

extern float rollErr;
extern float rollDot;
extern float rollCommand;
extern float pitchErr;
extern float pitchDot;
extern float pitchCommand;
extern float yawErr;
extern float yawDot;
extern float yawCommand;

extern signed int ACCEL_XOUT;
extern signed int ACCEL_YOUT;
extern signed int ACCEL_ZOUT;
extern float GYRO_XRATE;
extern float GYRO_YRATE;
extern float GYRO_ZRATE;
extern int GYRO_XRATERAW;
extern int GYRO_YRATERAW;
extern int GYRO_ZRATERAW;

extern unsigned char GYRO_XOUT_L;
extern unsigned char GYRO_XOUT_H;
extern unsigned char GYRO_YOUT_L;
extern unsigned char GYRO_YOUT_H;
extern unsigned char GYRO_ZOUT_L;
extern unsigned char GYRO_ZOUT_H;
extern signed int GYRO_XOUT;
extern signed int GYRO_YOUT;
extern signed int GYRO_ZOUT;

extern unsigned char ACCEL_XOUT_L;
extern unsigned char ACCEL_XOUT_H;
extern unsigned char ACCEL_YOUT_L;
extern unsigned char ACCEL_YOUT_H;
extern unsigned char ACCEL_ZOUT_L;
extern unsigned char ACCEL_ZOUT_H;

extern signed long GYRO_XOUT_OFFSET_1000SUM;
extern signed long GYRO_YOUT_OFFSET_1000SUM;
extern signed long GYRO_ZOUT_OFFSET_1000SUM;

extern float GYRO_XANGLE;
extern float GYRO_YANGLE;
extern float GYRO_ZANGLE;
extern long GYRO_XANGLERAW;
extern long GYRO_YANGLERAW;
extern long GYRO_ZANGLERAW;
extern float ACCEL_XANGLE;
extern float ACCEL_YANGLE;
extern float ACCEL_ZANGLE;

extern float KALMAN_XANGLE;
extern float KALMAN_YANGLE;
extern float KALMAN_ZANGLE;

extern signed int GYRO_XOUT_OFFSET;
extern signed int GYRO_YOUT_OFFSET;
extern signed int GYRO_ZOUT_OFFSET;

extern float OC1_output;
extern float OC2_output;
extern float OC3_output;
extern float OC4_output;

extern float XINTEGRAL;
extern float YINTEGRAL;

extern int throttle_input;
extern int yaw_input;
extern int pitch_input;
extern int roll_input;


#ifdef	__cplusplus
}
#endif

#endif	/* COMMON_H */

