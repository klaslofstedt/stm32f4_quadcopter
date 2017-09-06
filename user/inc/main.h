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

pid_data_t pid_roll = {
    .input = 0,
    .last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0.0f,
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .last_error = 0,
	.output = 0,
    .k_p = 0.006,//0.00676f, // 0.006 // 0.0062
    .k_i = 0, //0.00008f, // 0.0005
    .k_d = 0.9//0.00045f // 0.0009 // 0.00152
};

pid_data_t pid_pitch = {
    .input = 0,
	.last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0.0f,
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .last_error = 0,
	.output = 0,
    .k_p = 0.006,//0.00676f,
    .k_i = 0, //0.00008f,
    .k_d = 0.9//0.00045f 
};

pid_data_t pid_yaw = {
    .input = 0,
	.last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0.0f,
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .last_error = 0,
	.output = 0,
    .k_p = -0.001, //-0.002
    .k_i = 0.0f,
    .k_d = 0.0
};

pid_data_t pid_altitude = {
    .input = 0,
	.last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0.0f,
    .boundary_max = 1.0f,
    .boundary_min = 0,
    .last_error = 0,
	.output = 0,
    .k_p = 0.0f,//0.0227,
    .k_i = 0.0f,
    .k_d = 0.0f//2.885
};

joystick_data_t joystick_roll = {
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

joystick_data_t joystick_pitch = {
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

joystick_data_t joystick_yaw = {
    .pin_num = 9,
    .freq_desired = 55,
    .freq_input = 0,
    .freq_accuracy = 1,
    .duty_max = 11.15,
    .duty_min = 5.8,
    .duty_center = 8.43,
    .duty_thresh = 0.15,
    .duty_input = 0,
    .scalefactor = 100, // TODO: calculate this instead for set on init
    .output = 0,
    .output_limit = 200 // TODO: measure an actual max rate
};

joystick_data_t joystick_thrust = {
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

joystick_data_t joystick_toggle = {
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