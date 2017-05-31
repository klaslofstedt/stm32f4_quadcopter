#include "stm32f4xx_gpio.h"
#include "buzzer.h"
#include "pwm.h"

#define BUZZER_FREQUENCY_0 800
#define BUZZER_FREQUENCY_1 400
#define BUZZER_FREQUENCY_2 200
#define BUZZER_FREQUENCY_3 50
#define BUZZER_FREQUENCY_4 10


void buzzer_set(uint16_t duty)
{
    TIM13->CCR1 = duty;
}

void buzzer_init(void)
{
    // gpio
    GPIO_InitTypeDef  GPIO_InitStruct;
   
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);

    
    // timebase
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
    
    uint16_t prescaler = pwm_get_prescaler();

    TIM_BaseStruct.TIM_Prescaler = prescaler - 1;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period = (((SystemCoreClock / 2) / BUZZER_FREQUENCY_3) / prescaler) -1;

    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM13, &TIM_BaseStruct);
    TIM_Cmd(TIM13, ENABLE);
    
    TIM_OCInitTypeDef TIM_OCStruct;
    
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC1Init(TIM13, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Enable);
    
    TIM_OC2Init(TIM13, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM13, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM13, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM13, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM13, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM13, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM13, ENABLE);
    TIM_Cmd(TIM13, ENABLE);
}