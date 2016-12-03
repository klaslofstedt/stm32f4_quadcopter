#include <stdint.h>

#include "stm32f4xx_conf.h"

#include "esc.h"
#include "pwm.h"
#include "freertos_time.h"

#define ESC_INIT_LOW 0
#define ESC_INIT_HIGH 850
#define ESC_THRUST 0
#define ESC_RUN_MIN 1173
#define ESC_RUN_MAX 1860


void esc_set_speed(esc_t *esc)
{
#ifdef DEBUG
    printf2(" duty: %.4f ", esc->speed); 
#endif
	pwm_set_duty_cycle(esc->pin_number, esc->run_min + (uint16_t)(esc->speed * (esc->run_max - esc->run_min)));
}

void esc_init(esc_t *esc, uint16_t pin)
{
	uint16_t prescaler = pwm_get_prescaler();
	esc->pin_number = pin;
	esc->speed = 0;
	// devided by 1000000 because ESC data is in the base of us instead of s
	esc->run_min = (((SystemCoreClock/1000)/ (2 * prescaler)) * (ESC_INIT_HIGH));
	delay_ms(500);
	esc_set_speed(esc);
	delay_ms(2500);
    
	esc->run_min = (((SystemCoreClock/1000)/ (2 * prescaler)) * (ESC_RUN_MIN));
	esc->run_max = (((SystemCoreClock/1000)/ (2 * prescaler)) * (ESC_RUN_MAX));
	esc->lift_quad_min = 0;
}

void esc_check_bondaries(esc_t *esc)
{
	if(esc->speed > 1.0){
		esc->speed = 1.0;
	}else if(esc->speed < 0){
		esc->speed = 0;
	}
}
