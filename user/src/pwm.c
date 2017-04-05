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

RCC_ClocksTypeDef RCC_Clocks;

void pwm_input_init(TIM_TypeDef *TIMx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) 
{
	uint32_t ahbPeripheralPort;
	uint8_t gpioAlternateFunction;
	uint8_t nvicInterruptChannel;
    uint8_t hclckDivisor;
    uint8_t GPIO_PinSource;

	/* Note all of these timers are on AHB1, whichs means they run at 80,000,000 Hz. */
	if(TIMx == TIM2) {
		ahbPeripheralPort = RCC_AHB1Periph_GPIOA;
		gpioAlternateFunction = GPIO_AF_TIM2;
		nvicInterruptChannel = TIM2_IRQn;
        GPIO_PinSource = GPIO_PinSource15;
		/* AHB1 Peripherals run at half the HCLK Speed */
	    hclckDivisor = 2.0f;

	    /* Enable the timer clock */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	} if(TIMx == TIM3) {
		ahbPeripheralPort = RCC_AHB1Periph_GPIOB;
		gpioAlternateFunction = GPIO_AF_TIM3;
		nvicInterruptChannel = TIM3_IRQn;
        GPIO_PinSource = GPIO_PinSource4;
		/* AHB1 Peripherals run at half the HCLK Speed */
	    hclckDivisor = 2.0f;

	    /* Enable the timer clock */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	} else if (TIMx == TIM5) {
		ahbPeripheralPort = RCC_AHB1Periph_GPIOA;
		gpioAlternateFunction = GPIO_AF_TIM5;
		nvicInterruptChannel = TIM5_IRQn;
        GPIO_PinSource = GPIO_PinSource0;

		/* AHB1 Peripherals run at half the HCLK Speed */
	    hclckDivisor = 2.0f;

	    /* Enable the timer clock */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	} else if (TIMx == TIM9) {
		ahbPeripheralPort = RCC_AHB1Periph_GPIOE;
		gpioAlternateFunction = GPIO_AF_TIM9;
		nvicInterruptChannel = TIM1_BRK_TIM9_IRQn;
        GPIO_PinSource = GPIO_PinSource5;

		/* AHB2 Peripherals run at HCLK Speed */
	    hclckDivisor = 1.0f;

	    /* Enable the timer clock */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

	} else if (TIMx == TIM12) {
		ahbPeripheralPort = RCC_AHB1Periph_GPIOB;
		gpioAlternateFunction = GPIO_AF_TIM12;
		nvicInterruptChannel = TIM8_BRK_TIM12_IRQn;
        GPIO_PinSource = GPIO_PinSource14;

		/* AHB1 Peripherals run at half the HCLK Speed */
	    hclckDivisor = 2.0f;

	    /* Enable the timer clock */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	} else {
		// fail silently
	}

	/* Work out the system / bus / timer clock speed */
    RCC_GetClocksFreq(&RCC_Clocks);

    /* Enable the clock to the GPIO Port */
    RCC_AHB1PeriphClockCmd(ahbPeripheralPort, ENABLE);

    /* Turn on PB06, it will be connected to Timer 4, Channel 1.
     * Timer Channel 2 will also be used, I believe this renders pin PB7 unusable.
     */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    /* Connect TIM pin to AF2 */
    GPIO_PinAFConfig(GPIOx, GPIO_PinSource, gpioAlternateFunction);

    /* init the timer:
     * It doesn't really matter what prescaler we use, because the duty cycle is calculated as a percentage.
     *    (as long as the prescalar ensures that the counter will not overflow)
     * The maximum (16bit) period should never be reached, as we will reset the counter before we get there.
     */
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 1000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 65535; // change this?
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &timerInitStructure);

	/* Enable the Timer counter */
	TIM_Cmd(TIMx, ENABLE);

	/* We're attempting to not have a prescalar, that is, not divide the incoming signal.
	 * I wonder this prescalar differs from the above prescalar.
	 * Channel 1 is configured to capture on the rising edge.
	 * */
	TIM_ICInitTypeDef TIM_ICInitStructure1;
	TIM_ICInitStructure1.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure1.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure1.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure1.TIM_ICPrescaler = 0;
	TIM_ICInitStructure1.TIM_ICFilter = 0;
	TIM_ICInit(TIMx, &TIM_ICInitStructure1);

	/*
	 * Channel 2 is configured to capture on the falling edge.
	 * */
	TIM_ICInitTypeDef TIM_ICInitStructure2;
	TIM_ICInitStructure2.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure2.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure2.TIM_ICSelection = TIM_ICSelection_IndirectTI;
	TIM_ICInitStructure2.TIM_ICPrescaler = 0;
	TIM_ICInitStructure2.TIM_ICFilter = 0;
	TIM_ICInit(TIMx, &TIM_ICInitStructure2);

	/* Ensure that Channel two is set up as a slave, and that it resets the counters on a falling edge */
	TIM_SelectInputTrigger(TIMx, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIMx, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIMx, TIM_MasterSlaveMode_Enable);

	/* Enable the interrupt that gets fired when the timer counter hits the period */
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);

	/* Enable the Timer interrupts */
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = nvicInterruptChannel;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

void pwm_output_init(void)
{
    pwm_gpio_init();
	pwm_timebase_init();
	pwm_output_compare_init();
}

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

uint32_t pwm_get_duty_cycle(uint8_t PinNum)
{
    uint32_t duty = 0;
    switch(PinNum){
    case 12:
        duty = TIM4->CCR1;
        //printf2(" %d %d", 12, TIM4->CCR1);
        break;
        
    case 13:
        duty = TIM4->CCR2;
        //printf2(" %d %d", 13, TIM4->CCR2);
        break;
        
    case 14:
        duty = TIM4->CCR3;
        //printf2(" %d %d", 14, TIM4->CCR3);
        break;
        
    case 15:
        duty = TIM4->CCR4;
        //printf2(" %d %d\n\r", 15, TIM4->CCR4);
        break;
        
    default:
        // do nothing
        break;
    }
    return duty;
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
    printf2("Prescaler: %d\n\r", prescaler);
    TIM_BaseStruct.TIM_Prescaler = prescaler - 1;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period = (((SystemCoreClock / 2) / PWM_FREQUENCY) / prescaler) -1;
    printf2("Period: %d\n\r", TIM_BaseStruct.TIM_Period);
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
