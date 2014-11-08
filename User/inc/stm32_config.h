#ifndef STM32_CONFIG_H
#define STM32_CONFIG_H

#include "stm32f10x.h"

void SysTick_Init(void);              //1 ms 中断初始化
void RCC_Configuration(void);         //开启PA，PB，PB，USART1，T1,T2,T3,T4
void GPIO_Configuration(void);        //PC0_PC12输出高电平触发，PA0\1  PA2\3  PA6\7,PB6\7捕获高电平  //PB0\1\2\3选通echo,隔离不工作探头的Echo
void NVIC_Configuration(void);        //设置中断优先级
void USART_Configuration(void);       //配置串口
void Encoder_Configration(void);
void TIM8_PWM_Output(void);

#endif