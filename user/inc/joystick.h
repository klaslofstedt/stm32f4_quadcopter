#ifndef JOYSTICK_H
#define JOYSTICK_H

// FreeRTOS kernel includes
#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "semphr.h"
#include "queue.h"


typedef struct{
    uint8_t pin_num;
	float freq_desired;
    float freq_input;
    float freq_accuracy;
    float duty_max;
    float duty_min;
    float duty_center;
    float duty_thresh;
    float duty_input;
    float scalefactor; // TODO: calculate this instead for set on init
    float output;
    float output_limit;
    float avg[7];
} joystick_data_t;

float joystick_read_setpoint(joystick_data_t* in);
float joystick_read_thrust(joystick_data_t* in);
float joystick_read_toggle(joystick_data_t* in);

#endif