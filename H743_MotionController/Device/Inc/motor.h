#ifndef __MOTOR_H
#define __MOTOR_H

#include "autocontrol.h"

#define Motor_Pwm_Change_Speed_HIGH 160
#define Motor_Pwm_Change_Speed_LOW 40
#define Motor_Pwm_Median_Duty 3000
#define Motor_Pwm_Half_Range 1000
#define Motor_Pwm_Half_Range_N 1000
#define Motor_Pwm_Half_Range_P 720

typedef struct {
    float pwm[4];
    float thrust[4];
} ThrustCurve;


extern ThrustCurve thrustcurve[6];

void ThrustCurveInit(ThrustCurve *thrustcurve);
void MotorInit(void);
void MotorPwmRefresh(float *motorthrust);
void UploadThrustCurveData(int motor_num);
void UploadCurrentPWMOutput(void);

#endif