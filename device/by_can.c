/**
  *****************************************************************************
  * @title		can.c
  * @platform	STM32F103
  * @author		v.simonenko
  * @version	V1.1.0
  * @date		06.04.2021
  *******************************************************************************
*/

#include "by_can.h"
#include "generic.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"


uint8_t CAN1_Init(uint16_t CANbitRate)
{
	CAN_DeInit(CAN1);

    /* CAN GPIOs configuration */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_CAN1_GPIO_Periph, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    // CAN RX pin
    GPIO_InitStructure.GPIO_Pin = CAN1_RX_SOURCE;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

    // CAN TX pin
    GPIO_InitStructure.GPIO_Pin = CAN1_TX_SOURCE;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

#ifdef CAN1_ReMap
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE); // CAN1 remap PB8, PB9
#endif

    CAN_InitTypeDef CAN_InitStructure;
	CAN_StructInit(&CAN_InitStructure);

	switch (CANbitRate)
	{
		case 1000: CAN_InitStructure.CAN_Prescaler = 2;
			break;
		case 500: CAN_InitStructure.CAN_Prescaler = 4;
			break;
		default:
		case 250: CAN_InitStructure.CAN_Prescaler = 8;
			break;
		case 125: CAN_InitStructure.CAN_Prescaler = 16;
			break;
		case 100: CAN_InitStructure.CAN_Prescaler = 20;
			break;
		case 50: CAN_InitStructure.CAN_Prescaler = 40;
			break;
		case 20: CAN_InitStructure.CAN_Prescaler = 100;
			break;
		case 10: CAN_InitStructure.CAN_Prescaler = 200;
			break;
	}
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_15tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;

    return CAN_Init(CAN1, &CAN_InitStructure);
}

void CAN1_Filter_Init()
{
    // CAN filter init
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0b0000000000011111; // mask extended ids
	CAN_FilterInitStructure.CAN_FilterIdHigh =  0x0;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0b1111111111111000; // mask extended ids
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

void CAN1_IT_Init()
{
    // CAN FIFO0 message pending interrupt enable
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

uint32_t CAN_IsMailboxFree(CAN_TypeDef *CANptr)
{
	return (CANptr->TSR & CAN_TSR_TME) != 0;
}
