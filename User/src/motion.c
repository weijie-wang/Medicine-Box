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

#include "motion.h"

/*
 * Calculate the Motor PWMIN with PID
 * Para-----"*Motor" is the pointer to the Motor parameter structure
 * Para-----"*MotorPID" is the pointer to the Motor PID parameter struture
 * Return---status: 0 is success ;-1 is false
*/
int32_t CalcSpeedPID(MOTOR* Motor,PIDPARA* MotorPID)
{
		MotorPID->LastError = MotorPID->Error;
		MotorPID->Error = Motor->VelocityExpect - Motor->VelocityCurrent;
		MotorPID->Integral += MotorPID->Error;
		MotorPID->Differ = (MotorPID->Error -MotorPID->LastError) / Motor->SampleTime;
	
		Motor->INPWM = MotorPID->Error*MotorPID->Kp + MotorPID->Integral*MotorPID->Ki + MotorPID->Differ*MotorPID->Kd;
		Motor->INPWM = (Motor->INPWM <= Motor->INPWM_Uplimit) && (Motor->INPWM >= Motor->INPWM_Lowlimit) ? \
										Motor->INPWM : \
									  (Motor->INPWM > Motor->INPWM_Uplimit ? Motor->INPWM_Uplimit : Motor->INPWM_Lowlimit);
		return Motor->INPWM;
										
}
/*
 * Calculate the Motor velocity expected with PID
 * Para-----"*Position" is the pointer to the Motor Position parameter structure
 * Para-----"*PositionPID" is the pointer to the Motor Position PID parameter struture
 * Return---status: 0 is success ;-1 is false
*/
int CalcPositionPID(POSITION *Position,PIDPARA *PositionPID,MOTOR* Motor)
{
		Position->PositionCurrent += Motor->VelocityCurrent * Motor->SampleTime;
		PositionPID->LastError = PositionPID->Error;
		PositionPID->Error = Position->PositionExpect - Position->PositionCurrent;
		PositionPID->Integral += PositionPID->Error;
		PositionPID->Differ = (PositionPID->Error - PositionPID->LastError) / Motor->SampleTime;
	
		Position->VelocityExpect = PositionPID->Error*PositionPID->Kp \
														 + PositionPID->Integral*PositionPID->Ki \
														 + PositionPID->Differ*PositionPID->Kd;
	
		Motor->VelocityExpect = Position->VelocityExpect;
	
		return 0;
		
}
/*
 * Detect the Motor velocity current
 * Para-----"*Motor" is the pointer to the Motor parameter structure,the results will be sent to 
 * Para-----"TIMx" indicate the Encoder Timer port
 * Para-----"time" is sample time 
 * Return---status: 0 is success ;-1 is false
*/
int DetectVelocity(MOTOR *Motor,TIM_TypeDef* TIMx)
{    
	
		int32_t EncoderDelta;
    Motor->EncoderPrevious = Motor->EncoderCurrent;
    Motor->EncoderCurrent = TIM_GetCounter(TIMx);            //编码器的读数
    
		if( (Motor->EncoderCurrent < 0x2000) && (Motor->EncoderPrevious > 0xd000) )           // 超过上界//
			{
				EncoderDelta = Motor->EncoderCurrent - Motor->EncoderPrevious + 0x10000;   //正转//                                
	   	}
	  else if( (Motor->EncoderCurrent > 0xd000) && (Motor->EncoderPrevious < 0x2000) )    // 超过下界//
	   	{
				EncoderDelta= -(Motor->EncoderPrevious - Motor->EncoderCurrent + 0x10000); //正转//                                                                               
	   	}		
	  else if( Motor->EncoderCurrent > Motor->EncoderPrevious )
	   	{
				EncoderDelta = Motor->EncoderCurrent - Motor->EncoderPrevious;             //正转//
	   	}
	  else
	   	{
				EncoderDelta= -(Motor->EncoderPrevious - Motor->EncoderCurrent);           //反转//                                                     
	   	} 
		
		Motor->VelocityCurrent = EncoderDelta / Motor->SampleTime;
			
		return 0;
	  
}




