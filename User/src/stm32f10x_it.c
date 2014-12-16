/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "motion.h"
#include <math.h>
#include <string.h>

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
		int32_t temp ;
	  int i;


		flag0 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0);
	
		if(flag0 == 0)                 //光电开关没有检测到药片
    {
			count_Up0 = 0;
			count_Down0 ++;
			if(count_Down0 > 5)
      {
				count_Down0 = 0;
				down_flag0 = 0;           
      }            
    }       
    else                 //光电开关检测到有药片
    {
			count_Up0 ++;
			count_Down0 = 0;
			if(count_Up0 > 5)
      {
				count_Up0 = 0;
				if(down_flag0 == 0)
				{
					down_flag0 = 1;
					motor_up.VelocityExpect -= 10;
        }  
		  }			
    }
		
		flag1 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2);
	
		if(flag1 == 0)                 //光电开关没有检测到药片
    {
			count_Up1 = 0;
			count_Down1 ++;
			if(count_Down1 > 5)
      {
				count_Down1 = 0;
				down_flag1 = 0;           
      }            
    }       
    else                 //光电开关检测到有药片
    {
			count_Up1 ++;
			count_Down1 = 0;
			if(count_Up1 > 5)
      {
				count_Up1 = 0;
				if(down_flag1 == 0)
				{
					down_flag1 = 1;
					motor_up.VelocityExpect += 10;
        }  
		  }			
    }
		/**********************************DOWN MOTOR POSITION CONTROL*********************************************/
	
		flag = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
	
		if(flag == 0)                 //光电开关没有检测到药片
    {
			count_Up = 0;
			count_Down ++;
			if(count_Down > 5)
      {
				count_Down = 0;
				down_flag = 0;           
      }            
    }       
    else                 //光电开关检测到有药片
    {
			count_Up ++;
			count_Down = 0;
			if(count_Up > 5)
      {
				count_Up = 0;
				if(down_flag == 0)
				{
					down_flag = 1;
					n++;
					if(n >= med_num) {
						m += cell_num;
						n = 0;
					}
        }  
		  }			
    }
		
		
		i = (long long)position_down.PositionCurrent / interval;
		position_down.PositionCurrent -= i * interval;
		m -= i;
		position_down.PositionExpect = m * interval;
		DetectVelocity(&motor_down , TIM2);
		CalcPositionPID(&position_down , &positionPID_down , &motor_down);
		temp = CalcSpeedPID(&motor_down , &motorPID_down);	
		if(temp>=0)
		{
			temp = temp;
			GPIO_SetBits(GPIOB, GPIO_Pin_10);
			GPIO_ResetBits(GPIOB, GPIO_Pin_11);
		}
	  else
		{
			temp = -temp;
			GPIO_SetBits(GPIOB, GPIO_Pin_11);
			GPIO_ResetBits(GPIOB, GPIO_Pin_10);
		}
	  TIM4->CCR2 = temp;
		
		
		/******************************* UP MOTOR VELOCITY CONTROL ***************************************/
	
		DetectVelocity(&motor_up , TIM3);
		temp = CalcSpeedPID(&motor_up , &motorPID_up);	
		if(temp>=0)
		{
			temp = temp;
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		}
	  else
		{
			temp = -temp;
			GPIO_SetBits(GPIOB, GPIO_Pin_13);
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		}	
	  TIM4->CCR1 = temp;
		
}

void TIM4_IRQHandler(void)
{

	/*Group_4*/

	
}

void TIM3_IRQHandler(void)
{

	/*Group_3*/
	
	
}

void TIM2_IRQHandler(void)
{
	/*Group_2*/


	/*Group_1*/

	
}

void TIM1_CC_IRQHandler(void)
{
}

void USART1_IRQHandler(void)
{
    int data;
		int ten, unit;
		uint8_t Rx;
	  static uint8_t Rx_count=0;
    static RxIT_Flag_t  RxIT_Flag=RxIT_Flag_NoFrame;
    static RxIT_Rcv_t RxIT_Rcv=RxIT_Rcv_NoData;
    Rx=USART_ReceiveData(USART1);
	  USART_SendData(USART1,Rx);
    switch(RxIT_Flag)
    {
        case RxIT_Flag_NoFrame:
        if(Rx == 0xEE)
        {
         RxIT_Flag=RxIT_Flag_Header; 
         RxIT_Rcv = RxIT_Rcv_NoData;
         Rx_count=0;
        }else
        {
         RxIT_Rcv=RxIT_Rcv_NoData;
         RxIT_Flag=RxIT_Flag_NoFrame;  
         Rx_count=0; 
         return;
        }
        break;
        
       case RxIT_Flag_Header:
       if(Rx == 0xAA)
       {
        RxIT_Flag=RxIT_Flag_Payload;
        Rx_count=1;
       }else
        {
           RxIT_Rcv = RxIT_Rcv_NoData;
           RxIT_Flag=RxIT_Flag_NoFrame;
           Rx_count=0;
           return;
        }
        break;
        
       case RxIT_Flag_Payload:
       Rx_count++;
       break;
        
      default:
      RxIT_Rcv = RxIT_Rcv_NoData;  
      RxIT_Flag = RxIT_Flag_NoFrame;
      Rx_count = 0;
      return;
      
    }
    RxBuffer[Rx_count]=Rx;
    
    if(Rx_count>=RxBufferSize-1)
    {
        if((RxBuffer[RxBufferSize-2]==0x00) && (RxBuffer[RxBufferSize-1]==0xBB) )
        {										
          RxIT_Rcv = RxIT_Rcv_Pending;
          RxIT_Flag = RxIT_Flag_NoFrame;
		      Rx_count = 0;
        }else
            {
              RxIT_Rcv = RxIT_Rcv_NoData;
              RxIT_Flag = RxIT_Flag_NoFrame;
    		      Rx_count = 0;
							motor_up.VelocityExpect = 0;
							interval = 0;
            }
        
    }
   
   if(RxIT_Rcv == RxIT_Rcv_Pending)
      {
        med_num = RxBuffer[2];
				cell_num = RxBuffer[3];
				if(cell_num <= 2) {
					positionPID_down.Kp = 0.0150;
					positionPID_down.Ki = 0.0000001;
					positionPID_down.Kd = 0.5;
				} else if(cell_num == 3){
					positionPID_down.Kp = 0.0100;
					positionPID_down.Ki = 0.0000001;
					positionPID_down.Kd = 0.5;
				} else if(cell_num == 4){
					positionPID_down.Kp = 0.00650;
					positionPID_down.Ki = 0.0000001;
					positionPID_down.Kd = 0.5;
				} else if(cell_num > 4){
					positionPID_down.Kp = 0.00450;
					positionPID_down.Ki = 0.0000001;
					positionPID_down.Kd = 0.5;
				}
				
				
				unit = RxBuffer[5] & 0x0f;
				ten = (RxBuffer[5]>>4) & 0x0f;
				data = ten *10 + unit;
				
				unit = RxBuffer[6] & 0x0f;
				ten = (RxBuffer[6]>>4) & 0x0f;
				data = ten *10 + unit + data * 100;
				
				unit = RxBuffer[7] & 0x0f;
				ten = (RxBuffer[7]>>4) & 0x0f;
				data = ten *10 + unit + data * 100;
				
				if(0x00 == RxBuffer[4]) {
					interval = -data;
				} else if(0x01 == RxBuffer[4]) {
					interval = data;
				}
				
				//interval = 11810;
				unit = RxBuffer[9] & 0x0f;
				ten = (RxBuffer[9]>>4) & 0x0f;
				data = ten *10 + unit;
				if(0x00 == RxBuffer[8]) {
					motor_up.VelocityExpect = -data;
				} else if(0x01 == RxBuffer[8]) {
					motor_up.VelocityExpect = data;
				}
				RxIT_Rcv = RxIT_Rcv_NoData; 
      }
			

	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
