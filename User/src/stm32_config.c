#include "stm32_config.h"

void SysTick_Init(void)
{
	if(SysTick_Config(SystemCoreClock / 1000))
	{
		while(1);
	}
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}



void RCC_Configuration(void)
{
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 | RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM4, ENABLE);

}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_0|GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

	//GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
	//GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_RESET);
	
	
}







void NVIC_Configuration(void)					   					//???????
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				   //??1?,??3?

//	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	 

	USART_InitStructure.USART_BaudRate = 19200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART1, &USART_InitStructure);

}

void Encoder_Configration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
    
    /*****************     
    //      A?    B?
    //TIM1: PA8    PA9  tim1???
    //TIM2: PA0    PA1 	  ok
    //TIM3: PA6    PA7	  ok
    //TIM4: PB6    PB7
    ******************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_Init(GPIOA,&GPIO_InitStructure);
    

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    

    
  TIM_DeInit(TIM3);
  TIM_DeInit(TIM2);

    
    /* Timer configuration in Encoder mode */
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = 0xffff;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
   
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;//ICx_FILTER;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

    	
	TIM_Cmd(TIM3, ENABLE); 
  TIM_Cmd(TIM2, ENABLE); 

}

void TIM8_PWM_Output(void)
{
		  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		  TIM_OCInitTypeDef  TIM_OCInitStructure;  
		  
		  /* TIM8 Configuration ---------------------------------------------------
		   Generates 7 PWM signals with 4 different duty cycles:
		   TIM1CLK = 72 MHz, Prescaler = 0, TIM1 counter clock = 72 MHz
		   TIM1 frequency = TIM1CLK/(TIM1_Period + 1) = 20.0 KHz
		  - TIM1 Channe8_1 & Channel1N duty cycle = TIM1->CCR1 / (TIM1_Period + 1) = 50% 
		  - TIM1 Channe8_2 & Channel2N duty cycle = TIM1->CCR2 / (TIM1_Period + 1) = 37.5% 
		  - TIM1 Channe8_3 & Channel3N duty cycle = TIM1->CCR3 / (TIM1_Period + 1) = 25%
		  - TIM1 Channe8_4 duty cycle = TIM1->CCR4 / (TIM1_Period + 1) = 12.5% 
		  ----------------------------------------------------------------------- */  
		  /* Time Base configuration */
		  TIM_TimeBaseStructure.TIM_Prescaler = 0;
		  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		  TIM_TimeBaseStructure.TIM_Period = 5000;
		  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		
		  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	  //????????
		  
		  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
		  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
		  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		
		  TIM_OCInitStructure.TIM_Pulse =2500 ;
		
		  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
		  
		   TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		
		   TIM_OCInitStructure.TIM_Pulse = 2500;
		   TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		
		  //TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
		
		  //TIM_OC3Init(TIM8, &TIM_OCInitStructure);
		  //TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
		
		 // TIM_OC4Init(TIM8, &TIM_OCInitStructure);

		 TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
         TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
		
		 	 /* TIM8 counter enable */
		  TIM_Cmd(TIM4, ENABLE);
		
		  /* TIM1 Main Output Enable */
		  TIM_CtrlPWMOutputs(TIM4, ENABLE);
}
