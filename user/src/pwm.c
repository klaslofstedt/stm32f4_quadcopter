// Name everything either "structure" or "struct", not both as atm

#include <stdint.h>
#include <math.h>

#include "stm32f4xx_gpio.h"
#include "pwm.h"
#include "printf2.h"

#define PWM_FREQUENCY 400

static void pwm_gpio_init(void);
static void pwm_timebase_init(void);
static void pwm_output_compare_init(void);

void pwm_input_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // TIM2 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // GPIOB clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //Change here, to GPIOA
    
    // TIM2 chennel2 configuration : PB.03
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Connect TIM pin to AF2 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    
    // Enable the TIM2 global Interrupt
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // second lowest interrupt level
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // second lowest sub int level
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* ---------------------------------------------------------------------------
    TIM2 configuration: PWM Input mode
    The external signal is connected to TIM2 CH2 pin (PB.03)
    TIM2 CCR2 is used to compute the frequency value
    TIM2 CCR1 is used to compute the duty cycle value
    
    In this example TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1), since
    APB1 prescaler is set to 1.
    TIM2CLK = PCLK1 = HCLK = SystemCoreClock
    
    External Signal Frequency = SystemCoreClock / TIM2_CCR2 in Hz.
    External Signal DutyCycle = (TIM2_CCR1*100)/(TIM2_CCR2) in %.
    Note:
    SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
    Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
    function to update SystemCoreClock variable value. Otherwise, any configuration
    based on this variable will be incorrect.
    --------------------------------------------------------------------------- */
    TIM_ICInitTypeDef TIM_ICInitStructure;
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2 | TIM_Channel_3 | TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    
    TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);
    
    // Select the TIM2 Input Trigger: TI2FP2
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
    
    // Select the slave Mode: Reset Mode
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);
    
    // TIM enable counter
    TIM_Cmd(TIM2, ENABLE);
    
    // Enable the CC2 Interrupt Request
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
}


void pwm_init(void)
{
    pwm_gpio_init();
	pwm_timebase_init();
	pwm_output_compare_init();
}
uint16_t testT= 0;
void pwm_set_duty_cycle(uint8_t PinNum, uint16_t DutyCycle)
{
    switch(PinNum){
    case 12:
        TIM4->CCR1 = DutyCycle;
        //printf2(" %d %d", 12, TIM4->CCR1);
        break;
        
    case 13:
        TIM4->CCR2 = DutyCycle;
        //printf2(" %d %d", 13, TIM4->CCR2);
        break;
        
    case 14:
        TIM4->CCR3 = DutyCycle;
        //printf2(" %d %d", 14, TIM4->CCR3);
        break;
        
    case 15:
        TIM4->CCR4 = DutyCycle;
        //printf2(" %d %d\n\r", 15, TIM4->CCR4);
        break;
        
    default:
        // do nothing
        break;
    }
}

static void pwm_gpio_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
}

static void pwm_timebase_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    uint16_t prescaler = pwm_get_prescaler();
    printf2("prescaler: %d", prescaler);
    TIM_BaseStruct.TIM_Prescaler = prescaler - 1;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period = (((SystemCoreClock / 2) / PWM_FREQUENCY) / prescaler) -1;
    printf2("period: %d", TIM_BaseStruct.TIM_Period);
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    TIM_Cmd(TIM4, ENABLE);
}

void pwm_output_compare_init(void)
{
    TIM_OCInitTypeDef TIM_OCStruct;
    
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM4, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM4, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM4, ENABLE); // what is this?
    TIM_Cmd(TIM4, ENABLE);
}

uint16_t pwm_get_prescaler(void)
{
    /*  To be able to get a value lower than 65535 for TIM_Period, we need to find
    a prescaler. It can be found by:
    
    SystemCoreClock / 2 / PWM_FREQUENCY / Prescaler < 65535
    */
    uint32_t prescaler = 1;
    uint32_t TimerSize = (uint32_t)(pow(2, 8*sizeof(uint16_t)));
    //uint32_t test = pow(2, 8*sizeof(PWM_TIMER));
    while((SystemCoreClock / (2 * PWM_FREQUENCY * prescaler)) > TimerSize){
        prescaler++;
    }
    return (uint16_t)prescaler;
}
