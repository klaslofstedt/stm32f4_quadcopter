/*
* This program turns on the 4 leds of the stm32f4 discovery board
* one after another.
* It defines shortcut definitions for the led pins and
* stores the order of the leds in an array which is being
* iterated in a loop.
*
* This program is free human culture like poetry, mathematics
* and science. You may use it as such.
*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"  
#include "stm32f4xx_tim.h"  
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_usart.h"
#include <math.h>

//#include <stm32f4_discovery.h>


#define USE_DISCOVERY_BOARD

/*
I/O pins used to control the motor driver.
This for the STM32F4 Discovery board.
Channel 1 IN use channel 1/2/4 of TIM5 on PA0/1/3.
Channel 2 IN use channel 1/3/4 of TIM3 on PC6/8/9.
Channel 3 IN use channel 1/3/4 of TIM2 on PA15/PA2/PB11.
EN use pins PE7-15
*/
#define CHAN1_EN1 GPIO_Pin_7  /* Connect to pin 3 on L6234 DIP20 */
#define CHAN1_IN1 GPIO_Pin_0             /* pin 2 */
#define CHAN1_EN2 GPIO_Pin_8             /* pin 18 */
#define CHAN1_IN2 GPIO_Pin_1             /* pin 19 */
#define CHAN1_EN3 GPIO_Pin_9             /* pin 8 */
#define CHAN1_IN3 GPIO_Pin_3             /* pin 9 */

#define CHAN2_EN1 GPIO_Pin_10
#define CHAN2_IN1 GPIO_Pin_6
#define CHAN2_EN2 GPIO_Pin_11
#define CHAN2_IN2 GPIO_Pin_8
#define CHAN2_EN3 GPIO_Pin_12
#define CHAN2_IN3 GPIO_Pin_9

#define CHAN3_EN1 GPIO_Pin_13
#define CHAN3_IN1 GPIO_Pin_15
#define CHAN3_EN2 GPIO_Pin_14
#define CHAN3_IN2 GPIO_Pin_2
#define CHAN3_EN3 GPIO_Pin_15
#define CHAN3_IN3 GPIO_Pin_11             /* pin 9 */



/* This is apparently needed for libc/libm (eg. powf()). */
//int __errno;

static volatile uint32_t pwm1_pulse_width_1 = 0;
static volatile uint32_t pwm1_pulse_width_2 = 0;
static volatile uint32_t pwm1_pulse_width_3 = 0;
static volatile uint32_t pwm2_pulse_width_1 = 0;
static volatile uint32_t pwm2_pulse_width_2 = 0;
static volatile uint32_t pwm2_pulse_width_3 = 0;
static volatile uint32_t pwm3_pulse_width_1 = 0;
static volatile uint32_t pwm3_pulse_width_2 = 0;
static volatile uint32_t pwm3_pulse_width_3 = 0;

static const float F_PI = 3.141592654f;

static void delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}



static void setup_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin =
        CHAN1_EN1|CHAN1_EN2|CHAN1_EN3 |
            CHAN2_EN1|CHAN2_EN2|CHAN2_EN3 |
                CHAN3_EN1|CHAN3_EN2|CHAN3_EN3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}


static const uint32_t pwm_period = 84000000/50000;

static void setup_timer(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
        | GPIO_Pin_3 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
    
    TIM_TimeBaseStructure.TIM_Period = pwm_period-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}


static void setup_timer_interrupt(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}


static void enable_channels_1(void)
{
    GPIO_SetBits(GPIOE, CHAN1_EN1);
    GPIO_SetBits(GPIOE, CHAN1_EN2);
    GPIO_SetBits(GPIOE, CHAN1_EN3);
}


static void enable_channels_2(void)
{
    GPIO_SetBits(GPIOE, CHAN2_EN1);
    GPIO_SetBits(GPIOE, CHAN2_EN2);
    GPIO_SetBits(GPIOE, CHAN2_EN3);
}


static void enable_channels_3(void)
{
    GPIO_SetBits(GPIOE, CHAN3_EN1);
    GPIO_SetBits(GPIOE, CHAN3_EN2);
    GPIO_SetBits(GPIOE, CHAN3_EN3);
}


static const float damper = 0.7f;

/* Set channel PWMs, duty cycle of 0.0..1.0 (0..100%). */
static void set_channels_1(float duty1, float duty2, float duty3)
{
    pwm1_pulse_width_1 = damper*duty1*pwm_period;
    pwm1_pulse_width_2 = damper*duty2*pwm_period;
    pwm1_pulse_width_3 = damper*duty3*pwm_period;
}


static void set_channels_2(float duty1, float duty2, float duty3)
{
    pwm2_pulse_width_1 = damper*duty1*pwm_period;
    pwm2_pulse_width_2 = damper*duty2*pwm_period;
    pwm2_pulse_width_3 = damper*duty3*pwm_period;
}


static void set_channels_3(float duty1, float duty2, float duty3)
{
    pwm3_pulse_width_1 = damper*duty1*pwm_period;
    pwm3_pulse_width_2 = damper*duty2*pwm_period;
    pwm3_pulse_width_3 = damper*duty3*pwm_period;
}



void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM2->CCR1 = pwm3_pulse_width_1;
        TIM2->CCR3 = pwm3_pulse_width_2;
        TIM2->CCR4 = pwm3_pulse_width_3;
    }
}


void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM3->CCR1 = pwm2_pulse_width_1;
        TIM3->CCR3 = pwm2_pulse_width_2;
        TIM3->CCR4 = pwm2_pulse_width_3;
    }
}


void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        TIM5->CCR1 = pwm1_pulse_width_1;
        TIM5->CCR2 = pwm1_pulse_width_2;
        TIM5->CCR4 = pwm1_pulse_width_3;
    }
}


int main(void)
{
    static const uint32_t m_delay = 150000;
    
   
    setup_gpio();

    setup_timer();
    setup_timer_interrupt();
    
    set_channels_1(0.0f, 0.0f, 0.0f);
    set_channels_2(0.0f, 0.0f, 0.0f);
    set_channels_3(0.0f, 0.0f, 0.0f);
    enable_channels_1();
    enable_channels_2();
    enable_channels_3();
    delay(2000000);
    
    while (1)
    {
        uint32_t i;
        
        for (i= 0; i < m_delay; ++i)
        {
            uint32_t j = (i + m_delay/3) % m_delay;
            uint32_t k = (i + 2*m_delay/3) % m_delay;
            set_channels_1(0.5f*(1.0f+sinf((2.0f*F_PI)*((float)i/(float)m_delay))),
                           0.5f*(1.0f+sinf((2.0f*F_PI)*((float)j/(float)m_delay))),
                           0.5f*(1.0f+sinf((2.0f*F_PI)*((float)k/(float)m_delay))));
            set_channels_2(0.5f*(1.0f+sinf((2.0f*F_PI)*((float)i/(float)m_delay))),
                           0.5f*(1.0f+sinf((2.0f*F_PI)*((float)j/(float)m_delay))),
                           0.5f*(1.0f+sinf((2.0f*F_PI)*((float)k/(float)m_delay))));
            set_channels_3(0.5f*(1.0f+sinf((2.0f*F_PI)*((float)i/(float)m_delay))),
                           0.5f*(1.0f+sinf((2.0f*F_PI)*((float)j/(float)m_delay))),
                           0.5f*(1.0f+sinf((2.0f*F_PI)*((float)k/(float)m_delay))));
        }
    }
    
    return 0;
}