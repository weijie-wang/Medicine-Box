/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/


#ifndef MOTION_H
#define MOTION_H


#include <math.h>
#include <string.h>
#include "stm32f10x.h"

#ifndef PI
#define PI 3.1415926
#endif

 typedef struct motor
{
    int32_t EncoderPrevious;
    int32_t EncoderCurrent;
		int32_t SampleTime;
	
    int32_t VelocityExpect;     //'+' means positive direction  '-' means negative direction 
    int32_t VelocityCurrent;    //'+' means positive direction  '-' means negative direction
	
	  int32_t INPWM;
		int32_t INPWM_Uplimit;
		int32_t INPWM_Lowlimit;
	
}MOTOR;

typedef struct position
{
		double PositionExpect;
    double PositionCurrent;
	
    double VelocityExpect;

}POSITION;


typedef struct PIDpara
{
    float Kp;
    float Ki;
    float Kd;
    
    float Error;
    float LastError;
    float Integral;
    float Differ;
    
}PIDPARA;		
		




extern MOTOR motor_up , motor_down;
extern PIDPARA motorPID_up , motorPID_down;
extern POSITION position_down;
extern PIDPARA positionPID_down;
extern int32_t n;
extern uint8_t flag , down_flag , count_Up , count_Down;


int DetectVelocity(MOTOR* Motor,TIM_TypeDef* TIMx);
int32_t CalcSpeedPID(MOTOR* Motor,PIDPARA* MotorPID);
int CalcPositionPID(POSITION *Position,PIDPARA *PositionPID,MOTOR* Motor);
   

#endif





/* [] END OF FILE */
