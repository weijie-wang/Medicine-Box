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
	  int interval , i;


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
        }  
		  }			
    }
		
		
		i = (long long)position_down.PositionCurrent / interval;
		position_down.PositionCurrent -= i * interval;
		n -= i;
		interval = 3850;
		position_down.PositionExpect = n * interval;
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
	  uint8_t Rx,temp;
    Rx = USART_ReceiveData(USART1);
	  temp = Rx;
	  Rx = (temp >> 4)*0x100 + (Rx & 0x0F);
		motor_up.VelocityExpect = -Rx;
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
