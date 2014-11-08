
#include "motion.h"
#include "stdio.h"
#include "stm32_config.h"

 				
MOTOR motor_up , motor_down;
PIDPARA motorPID_up , motorPID_down;
POSITION position_up , position_down;
PIDPARA positionPID_up , positionPID_down;
int32_t n;
uint8_t flag , down_flag , count_Up , count_Down;
uint8_t flag0 , down_flag0 , count_Up0 , count_Down0;
uint8_t flag1 , down_flag1 , count_Up1 , count_Down1;

int main()
{    

	
		memset(&motor_up,0,sizeof(MOTOR));
		memset(&motor_down,0,sizeof(MOTOR));
		memset(&motorPID_up,0,sizeof(PIDPARA));
		memset(&motorPID_down,0,sizeof(PIDPARA));
		memset(&position_down,0,sizeof(POSITION));
		memset(&positionPID_down,0,sizeof(PIDPARA)); 
	  n = 0;
		
	
		motor_up.VelocityExpect = 0;
		motor_up.INPWM_Uplimit = 2400;
		motor_up.INPWM_Lowlimit = -2400;
		motor_up.SampleTime = 1;
		motorPID_up.Kp = 100;
		motorPID_up.Ki = 0.1;
		motorPID_up.Kd = 0;
	
	
		motor_down.INPWM_Uplimit = 2400;
		motor_down.INPWM_Lowlimit = -2400;
		motor_down.SampleTime = 1;
		motorPID_down.Kp = 100;
		motorPID_down.Ki = 0.1;
		motorPID_down.Kd = 0;
		position_down.PositionCurrent = 0;
		positionPID_down.Kp = 0.015;
		positionPID_down.Ki = 0.000001;
		positionPID_down.Kd = 0;
		
	
	
	
	  RCC_Configuration();
	  GPIO_Configuration();
		SysTick_Init();
		USART_Configuration();	
		NVIC_Configuration();
		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//开启串口接收中断
		USART_Cmd(USART1, ENABLE);
	  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //开系统定时中断
		Encoder_Configration();
		TIM8_PWM_Output();
							  	


	while(1)
	{	
		
			
	}	

   

}




								