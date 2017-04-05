#ifndef PWM_h
#define PWM_h

#include "stm32f4xx_gpio.h"

void pwm_input_init(TIM_TypeDef *TIMx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void pwm_output_init(void);

void pwm_set_duty_cycle(uint8_t PinNum, uint16_t DutyCycle);
uint16_t pwm_get_prescaler(void);
uint32_t pwm_get_duty_cycle(uint8_t PinNum);

#endif
