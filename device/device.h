/*
 * device.h
 *
 * Device/project description
 *
 */

#ifndef DEVICE_DEVICE_H_
#define DEVICE_DEVICE_H_
#include "stm32f10x_gpio.h"

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

//GPIO Defines
#define PIN_LED GPIO_Pin_13 //PC13	BluePill Green LED

#endif /* DEVICE_DEVICE_H_ */
