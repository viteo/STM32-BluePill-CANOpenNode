/*
 * device_config.c
 *
 *  Created on: sep 2021
 *      Author: v.simonenko
 */

#include "device.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

void Device_GPIO_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = PIN_LED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ResetBits(GPIOC, PIN_LED);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Device_Timer_Init()
{
	uint32_t TIM_ClockFreq;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	if (RCC_Clocks.HCLK_Frequency != RCC_Clocks.PCLK1_Frequency)
		TIM_ClockFreq = RCC_Clocks.PCLK1_Frequency * 2;
	else
		TIM_ClockFreq = RCC_Clocks.PCLK1_Frequency;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	// Periodic poll
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM_ClockFreq / 2 / (1000); // (frequency in kHz) might overflow when 72 MHz
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 2 * (100); // (preiod in ms)
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(TIM4_IRQn);

	// SysTick 1ms poll
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}
