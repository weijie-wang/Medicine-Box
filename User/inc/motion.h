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

#define RxBufferSize 12

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
		
typedef enum
{
	RxIT_Flag_NoFrame = 0,
	RxIT_Flag_Header,
	RxIT_Flag_Payload
} RxIT_Flag_t;

typedef enum
{
	RxIT_Rcv_NoData = 0,
	RxIT_Rcv_Pending
} RxIT_Rcv_t;



extern MOTOR motor_up , motor_down;
extern PIDPARA motorPID_up , motorPID_down;
extern POSITION position_down;
extern PIDPARA positionPID_down;
extern int32_t n, m;
extern uint8_t flag , down_flag , count_Up , count_Down;
extern uint8_t flag0 , down_flag0 , count_Up0 , count_Down0;
extern uint8_t flag1 , down_flag1 , count_Up1 , count_Down1;
extern int interval;
extern int data;
extern uint8_t RxBuffer[];
extern uint8_t RxRcv[];
extern int med_num;
extern int cell_num;

int DetectVelocity(MOTOR* Motor,TIM_TypeDef* TIMx);
int32_t CalcSpeedPID(MOTOR* Motor,PIDPARA* MotorPID);
int CalcPositionPID(POSITION *Position,PIDPARA *PositionPID,MOTOR* Motor);




#endif





/* [] END OF FILE */
