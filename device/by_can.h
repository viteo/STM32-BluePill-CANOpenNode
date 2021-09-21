/**
  *****************************************************************************
  * @title		can.h
  * @platform	STM32F103
  * @author		v.simonenko
  * @version	V1.2.0
  * @date		06.04.2021
  *****************************************************************************
*/

#ifndef __CAN
#define __CAN

#include "stm32f10x.h"
#include "stddef.h"

//#define CAN1_ReMap
#ifdef CAN1_ReMap
#define CAN1_GPIO_PORT GPIOB
#define CAN1_RX_SOURCE GPIO_Pin_8
#define CAN1_TX_SOURCE GPIO_Pin_9
#define RCC_CAN1_GPIO_Periph RCC_APB2Periph_GPIOB
#else
#define CAN1_GPIO_PORT GPIOA
#define CAN1_RX_SOURCE GPIO_Pin_11
#define CAN1_TX_SOURCE GPIO_Pin_12
#define RCC_CAN1_GPIO_Periph RCC_APB2Periph_GPIOA
#endif

uint8_t CAN1_Init(uint16_t CANbitRate);
void CAN1_Filter_Init();
void CAN1_IT_Init();
uint32_t CAN_IsMailboxFree(CAN_TypeDef *CANptr);

#endif //__CAN
