#include <stdint.h>

#include "stm32f4xx_conf.h"

#include "esc.h"
#include "pwm.h"
#include "freertos_time.h"
#include "uart.h"

#define ESC_INIT_LOW 0
#define ESC_INIT_HIGH 850
#define ESC_THRUST 0
#define ESC_RUN_MIN 1173
#define ESC_RUN_MAX 1860


void esc_set_speed(esc_t *esc, float speed)
{
    esc->speed = speed;
    if(esc->speed > esc->speed_max){
		esc->speed = esc->speed_max;
	}else if(esc->speed < esc->speed_min){
		esc->speed = esc->speed_min;
	}
	pwm_set_duty_cycle(esc->pin_number, (esc->run_min + (uint16_t)(esc->speed * (esc->run_max - esc->run_min))));
}

void esc_init(esc_t *esc)
{
	uint16_t prescaler = pwm_get_prescaler();
	float speed = 0;
	// devided by 1000000 because ESC data is in the base of us instead of s
    // This line is shitty since it's not actually init run_min but init the esc
	esc->run_min = (((SystemCoreClock/1000000) / (2 * prescaler)) * (ESC_INIT_HIGH));
    uart_printf("pin: %d pwm: %d\n\r", esc->pin_number, ((SystemCoreClock/1000000) / (2 * prescaler)) * (ESC_INIT_HIGH));
	delay_ms(500);
	esc_set_speed(esc, speed);
	delay_ms(2500);
    
	esc->run_min = (((SystemCoreClock/1000000) / (2 * prescaler)) * (ESC_RUN_MIN));
	esc->run_max = (((SystemCoreClock/1000000) / (2 * prescaler)) * (ESC_RUN_MAX));
	esc->lift_quad_min = 0;
}