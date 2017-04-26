#ifndef MAIN_H
#define MAIN_H

#include "imu.h"
#include "esc.h"
#include "pid.h"
#include "joystick.h"
#include "altitude.h"

esc_t esc1 = {
    .pin_number = 12,
    .speed_max = 1.0,
    .speed_min = 0
};

esc_t esc2 = {
    .pin_number = 13,
    .speed_max = 1.0,
    .speed_min = 0
};

esc_t esc3 = {
    .pin_number = 14,
    .speed_max = 1.0,
    .speed_min = 0
};

esc_t esc4 = {
    .pin_number = 15,
    .speed_max = 1.0,
    .speed_min = 0
};

pid_data_t roll = {
    .input = 0,
    .last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0,
	.output = 0,
    .k_p = 0.006,
    .k_i = 0.0,
    .k_d = 0.9 // kanske höja?
};

pid_data_t pitch = {
    .input = 0,
	.last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0,
	.output = 0,
    .k_p = 0.006,
    .k_i = 0.0,
    .k_d = 0.9
};

pid_data_t yaw = {
    .input = 0,
	.last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0,
	.output = 0,
    .k_p = -2.0,
    .k_i = 0.0,
    .k_d = 0.0
};

pid_data_t pid_altitude = {
    .input = 0,
	.last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0,
	.output = 0,
    .k_p = -2.0,
    .k_i = 0.0,
    .k_d = 0.0
};

joystick_data_t joystick_roll_g = {
    .pin_num = 5,
    .freq_desired = 55,
    .freq_input = 0,
    .freq_accuracy = 1,
    .duty_max = 11.155,
    .duty_min = 5.775,
    .duty_center = 8.27,
    .duty_thresh = 0.15,
    .duty_input = 0,
    .scalefactor = 10,
    .output = 0,
    .output_limit = 15
};

joystick_data_t joystick_pitch_g = {
    .pin_num = 2,
    .freq_desired = 55,
    .freq_input = 0,
    .freq_accuracy = 1,
    .duty_max = 11.089,
    .duty_min = 5.774,
    .duty_center = 8.53,
    .duty_thresh = 0.15,
    .duty_input = 0,
    .scalefactor = 10,
    .output = 0,
    .output_limit = 15
};

joystick_data_t joystick_yaw_g = {
    .pin_num = 9,
    .freq_desired = 55,
    .freq_input = 0,
    .freq_accuracy = 1,
    .duty_max = 11.15,
    .duty_min = 5.8,
    .duty_center = 8.43,
    .duty_thresh = 0.15,
    .duty_input = 0,
    .scalefactor = 0.1,
    .output = 0,
    .output_limit = 2 //????? fixa detta!
};

joystick_data_t joystick_thrust_g = {
    .pin_num = 12,
    .freq_desired = 55,
    .freq_input = 0,
    .freq_accuracy = 1,
    .duty_max = 11,
    .duty_min = 5.7,
    .duty_center = 8.35,
    .duty_thresh = 0.15,
    .duty_input = 0,
    .scalefactor = 0.188,
    .output = 0,
    .output_limit = 1 //????? fixa detta!
};

joystick_data_t joystick_toggle_g = {
    .pin_num = 3,
    .freq_desired = 55,
    .freq_input = 0,
    .freq_accuracy = 1,
    .duty_max = 11,
    .duty_min = 5.7,
    .duty_center = 8.35,
    .duty_thresh = 0.15,
    .duty_input = 0,
    .scalefactor = 0.1,
    .output = 0,
    .output_limit = 2 //????? fixa detta!
};



#endif