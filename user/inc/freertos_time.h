#ifndef __FREERTOS_TIME_H__
#define __FREERTOS_TIME_H__

#include <stdint.h>

#define SYSTICK_FREQUENCY   1000

void delay_ms(uint32_t delay_time);
int get_ms_count(unsigned long *count);
int get_tick_count(unsigned long *count);
 
#endif // __I2C_H__


