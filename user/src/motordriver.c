#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"  
#include "stm32f4xx_tim.h"  
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#include "motordriver.h"
#include "board.h"
#include <math.h>
#define M_PI acos(-1.0) // pi

static void setup_gpio(void);
static void setup_timer(void);
static void setup_timer_interrupt(void);
static void enable_drivers(void);

static volatile uint32_t pwm1_pulse_width_1 = 0;
static volatile uint32_t pwm1_pulse_width_2 = 0;
static volatile uint32_t pwm1_pulse_width_3 = 0;
static volatile uint32_t pwm2_pulse_width_1 = 0;
static volatile uint32_t pwm2_pulse_width_2 = 0;
static volatile uint32_t pwm2_pulse_width_3 = 0;
static volatile uint32_t pwm3_pulse_width_1 = 0;
static volatile uint32_t pwm3_pulse_width_2 = 0;
static volatile uint32_t pwm3_pulse_width_3 = 0;

static const uint32_t pwm_period = 84000000/50000;
static const float damper = 0.7f;

static volatile uint32_t m_delay = 400;
static volatile uint32_t i = 0;


void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        /*uint32_t j = (i + m_delay/3) % m_delay;
        uint32_t k = (i + 2*m_delay/3) % m_delay;
        set_channels_1(0.5f*(1.0f+sinf((2.0f*M_PI)*((float)i/(float)m_delay))),
                       0.5f*(1.0f+sinf((2.0f*M_PI)*((float)j/(float)m_delay))),
                       0.5f*(1.0f+sinf((2.0f*M_PI)*((float)k/(float)m_delay))));
        set_channels_2(0.5f*(1.0f+sinf((2.0f*M_PI)*((float)i/(float)m_delay))),
                       0.5f*(1.0f+sinf((2.0f*M_PI)*((float)j/(float)m_delay))),
                       0.5f*(1.0f+sinf((2.0f*M_PI)*((float)k/(float)m_delay))));
        set_channels_3(0.5f*(1.0f+sinf((2.0f*M_PI)*((float)i/(float)m_delay))),
                       0.5f*(1.0f+sinf((2.0f*M_PI)*((float)j/(float)m_delay))),
                       0.5f*(1.0f+sinf((2.0f*M_PI)*((float)k/(float)m_delay))));
        i++;
        if (i > m_delay){
            i = 0;
        }*/
        
        //TIM2->CCR1 = pwm3_pulse_width_1;
        //TIM2->CCR3 = pwm3_pulse_width_2;
        //TIM2->CCR4 = pwm3_pulse_width_3;
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

void motordriver_init(void)
{
    setup_gpio();
    setup_timer();
    setup_timer_interrupt();
    set_channels_1(0.0f, 0.0f, 0.0f);
    set_channels_2(0.0f, 0.0f, 0.0f);
    set_channels_3(0.0f, 0.0f, 0.0f);
    enable_drivers();
}

static void setup_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(CHANX_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = DRIVER1_EN | DRIVER2_EN | DRIVER3_EN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(CHANX_GPIO_PORT, &GPIO_InitStructure);
}


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
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,  GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,  GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,  GPIO_AF_TIM5);
    
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

static void enable_drivers(void)
{
    GPIO_SetBits(GPIOE, DRIVER1_EN);
    GPIO_SetBits(GPIOE, DRIVER2_EN);
    GPIO_SetBits(GPIOE, DRIVER3_EN);
}


/* Set channel PWMs, duty cycle of 0.0..1.0 (0..100%). */
void set_channels_1(float duty1, float duty2, float duty3)
{
    pwm1_pulse_width_1 = (uint32_t)(damper*duty1*(float)pwm_period);
    pwm1_pulse_width_2 = (uint32_t)(damper*duty2*(float)pwm_period);
    pwm1_pulse_width_3 = (uint32_t)(damper*duty3*(float)pwm_period);
}


void set_channels_2(float duty1, float duty2, float duty3)
{
    pwm2_pulse_width_1 = damper*duty1*pwm_period;
    pwm2_pulse_width_2 = damper*duty2*pwm_period;
    pwm2_pulse_width_3 = damper*duty3*pwm_period;
}


void set_channels_3(float duty1, float duty2, float duty3)
{
    pwm3_pulse_width_1 = damper*duty1*pwm_period;
    pwm3_pulse_width_2 = damper*duty2*pwm_period;
    pwm3_pulse_width_3 = damper*duty3*pwm_period;
}

void motordriver_set1(uint32_t i, uint32_t m_delay)
{
    uint32_t j = (i + m_delay/3) % m_delay;
    uint32_t k = (i + 2*m_delay/3) % m_delay;
    pwm1_pulse_width_1 = (uint32_t)(damper * pwm_period * 0.5f*(1.0f+sinf((2.0f*M_PI)*((float)i/(float)m_delay))));
    pwm1_pulse_width_2 = (uint32_t)(damper * pwm_period * 0.5f*(1.0f+sinf((2.0f*M_PI)*((float)j/(float)m_delay))));
    pwm1_pulse_width_3 = (uint32_t)(damper * pwm_period * 0.5f*(1.0f+sinf((2.0f*M_PI)*((float)k/(float)m_delay))));
}

