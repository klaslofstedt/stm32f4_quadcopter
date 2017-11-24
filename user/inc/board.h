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

// Motor driver L6234
#define CHANX_GPIO_PORT                 GPIOE
#define CHANX_GPIO_CLK                  RCC_AHB1Periph_GPIOE
#define DRIVER1_EN                      GPIO_Pin_7 // pin 3 on L6234
//#define CHAN1_IN1                       GPIO_Pin_0 // pin 2
#define DRIVER2_EN                      GPIO_Pin_8 // pin 18
//#define CHAN1_IN2                       GPIO_Pin_1 // pin 19
#define DRIVER3_EN                      GPIO_Pin_9 // pin 8
//#define CHAN1_IN3                       GPIO_Pin_3 // pin 9





#endif