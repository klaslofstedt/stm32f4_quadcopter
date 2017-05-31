/********************************************************************************
* @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
* @author  MCD Application Team
* @version V1.0.0
* @date    30-September-2011
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"  
#include "stm32f4xx_tim.h"  
#include "stm32f4xx_gpio.h" 
#include "imu.h"
#include "uart.h"
#include <stdint.h>

/** @addtogroup Template_Project
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t IC2Value = 0;
volatile uint16_t DutyCycle = 0;
volatile uint32_t Frequency = 0;

volatile float tim2_val = 0, tim3_val = 0, tim5_val = 0, tim9_val = 0, tim12_val = 0;//, ic4_val = 0;
volatile float tim2_duty = 0, tim3_duty = 0, tim5_duty = 0, tim9_duty = 0, tim12_duty = 0;//, ic4_duty = 0;
volatile float tim2_freq = 0, tim3_freq = 0, tim5_freq = 0, tim9_freq = 0, tim12_freq = 0;//, ic4_freq = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

float isr_read_freq(uint8_t tim)
{
    float temp = 0;
    switch(tim){
        case 2:
            temp = tim2_freq;
            break;
        case 3:
            temp = tim3_freq;
            break;
        case 5:
            temp = tim5_freq;
            break;
        case 9:
            temp = tim9_freq;
            break;
        case 12:
            temp = tim12_freq;
            break;
        default:
            temp = 0;
            break;
    }
    return temp;
}

float isr_read_duty(uint8_t tim)
{
    float temp = 0;
    switch(tim){
        case 2:
            temp = tim2_duty;
            break;
        case 3:
            temp = tim3_duty;
            break;
        case 5:
            temp = tim5_duty;
            break;
        case 9:
            temp = tim9_duty;
            break;
        case 12:
            temp = tim12_duty;
            break;
        default:
            temp = 0;
            break;
    }
    return temp;
}
/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
__weak void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
__weak void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
__weak void SysTick_Handler(void)
{
    
}

/**
* @brief  This function handles EXTI 3 interrupt request.
* @param  None
* @retval None
*/
__weak void EXTI9_5_IRQHandler(void)
{
}

/**
* @brief  This function handles EXTI 15-10 interrupt request.
* @param  None
* @retval None
*/
__weak void EXTI15_10_IRQHandler(void)
{
}


/**
* @brief  This function handles external interrupts generated by MPU.
* @param  None
* @retval None
*/

void EXTI4_IRQHandler(void)
{
    /* Handle new gyro*/
    gyro_data_ready_cb();
    EXTI_ClearITPendingBit(EXTI_Line4);
}


/**
* @brief  This function handles TIM4 global interrupt request.
* @param  None
* @retval None
*/
void TIM4_IRQHandler(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    
    // Clear TIM4 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
    
    // Get the Input Capture value
    IC2Value = TIM_GetCapture2(TIM4);
    
    if (IC2Value != 0)
    {
        // Duty cycle computation
        DutyCycle = (TIM_GetCapture1(TIM4) * 100) / IC2Value;
        
        // Frequency computation 
        //TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 
        
        Frequency = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value;
    }
    else
    {
        DutyCycle = 0;
        Frequency = 0;
    }
}


void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        // Get the Input Capture value
        tim2_val = TIM_GetCapture1(TIM2);
        if (tim2_val != 0)
        {
            //Duty cycle computation
            tim2_duty = (float)(TIM_GetCapture2(TIM2) * 100) / tim2_val;
            //Frequency computation
            tim2_freq = (float)SystemCoreClock / (float)(2 * 1000 * tim2_val);
        }
        else
        {
            tim2_duty = 0;
            tim2_freq = 0;
        }
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        // Get the Input Capture value
        tim3_val = TIM_GetCapture1(TIM3);
        if (tim3_val != 0)
        {
            //Duty cycle computation
            tim3_duty = (float)(TIM_GetCapture2(TIM3) * 100) / tim3_val;
            //Frequency computation
            tim3_freq = (float)SystemCoreClock / (float)(2 * 1000 * tim3_val);
        }
        else
        {
            tim3_duty = 0;
            tim3_freq = 0;
        }
    }
}

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        // Get the Input Capture value
        tim5_val = TIM_GetCapture1(TIM5);
        if (tim5_val != 0)
        {
            //Duty cycle computation
            tim5_duty = (float)(TIM_GetCapture2(TIM5) * 100) / tim5_val;
            //Frequency computation
            tim5_freq = (float)SystemCoreClock / (float)(2 * 1000 * tim5_val);
        }
        else
        {
            tim5_duty = 0;
            tim5_freq = 0;
        }
    }
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM12, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM12, TIM_IT_Update);
        // Get the Input Capture value
        tim12_val = (float)TIM_GetCapture1(TIM12);
        
        if (tim12_val != 0)
        {
            //Duty cycle computation
            tim12_duty = (float)(TIM_GetCapture2(TIM12) * 100) / tim12_val;
            //Frequency computation
            tim12_freq = (float)SystemCoreClock / (float)(2 * 1000 * tim12_val);
        }
        else
        {
            tim12_duty = 0;
            tim12_freq = 0;
        }
    }
}

void TIM1_BRK_TIM9_IRQHandler() {
	// makes sure the interrupt status is not reset (and therefore SET?)
    if (TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET) {
    	// ensure that the timer doesn't get triggered again
        TIM_ClearITPendingBit(TIM9, TIM_IT_Update);

        tim9_val = (float)TIM_GetCapture1(TIM9);
        
        if (tim9_val != 0)
        {
            //Duty cycle computation
            tim9_duty = (float)(TIM_GetCapture2(TIM9) * 100) / tim9_val;
            //Frequency computation
            tim9_freq = (float)SystemCoreClock / (float)(1 * 1000 * tim9_val);
        }
        else
        {
            tim9_duty = 0;
            tim9_freq = 0;
        }
    }
}

/*void TIM3_IRQHandler(void)
{

}*/

/*void TIM8_UP_TIM13_IRQHandler(void)
{

}
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{

}*/