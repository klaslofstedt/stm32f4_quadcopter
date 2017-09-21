#ifndef BOARD_H
#define BOARD_H

#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

// IMU interrupt pin
#define INVEN_INT_PIN                   GPIO_Pin_4
#define INVEN_INT_GPIO_PORT             GPIOA
#define INVEN_INT_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define INVEN_INT_EXTI_PORT             EXTI_PortSourceGPIOA
#define INVEN_INT_EXTI_PIN              EXTI_PinSource4
#define INVEN_INT_EXTI_LINE             EXTI_Line4
#define INVEN_INT_EXTI_IRQ              EXTI4_IRQn

// Debug GPIO pins
#define DEBUG_GPIO_PORT                 GPIOE
#define DEBUG_GPIO_CLK                  RCC_AHB1Periph_GPIOE
#define DEBUG_MAIN_TASK_PIN             GPIO_Pin_7
#define DEBUG_IMU_TASK_PIN              GPIO_Pin_8
#define DEBUG_ALT_TASK_PIN              GPIO_Pin_9
#define DEBUG_IMU_INT_PIN               GPIO_Pin_10
#define DEBUG_TEL_TASK_PIN              GPIO_Pin_11
#define DEBUG_IMU_ACT_PIN               GPIO_Pin_12
//#define DEBUG_GPIOE_13                  GPIO_Pin_13
//#define DEBUG_GPIOE_14                  GPIO_Pin_14
//#define DEBUG_GPIOE_15                  GPIO_Pin_15




#endif