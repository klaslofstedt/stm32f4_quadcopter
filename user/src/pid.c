#include "pid.h"

#define BOUNDARY_MIN -1.0f
#define BOUNDARY_MAX 1.0f

// Might wanna have a better error handling when using delta input as rate:
// https://github.com/Lauszus/LaunchPadFlightController/blob/fddbe4eb9303ea4301a714585d7383a2de275d80/src/PID.c

void pid_calc(pid_data_t* pid, unsigned long dt)
{
    // These are used for ease to read
    float p_term, i_term, d_term, error, output;
    
    // Calculate input rate with derivation of position instead from EKF output
    // *** IMPORTANT *** Does it need low pass-filter?                           
    pid->rate = (pid->input - pid->last_input) / (float)dt;
    
    // Calculate error between current and desired position
    error = pid->setpoint - pid->input;
    
    // Calculate the P contribution
    p_term = pid->k_p * error;

    // Calculate the I contribution
    pid->i_term += (float)(pid->k_i * (float)dt * error);
    if(pid->i_term > BOUNDARY_MAX){
        pid->i_term = BOUNDARY_MAX;
    }
    else if(pid->i_term < BOUNDARY_MIN) {
        i_term = BOUNDARY_MIN;
    }
    i_term = pid->i_term;

    // Calculate the D contribution
    d_term = pid->k_d * pid->rate;
    
    //Calculate output
    output = p_term + i_term - d_term;
    // Check boundaries
    if(output > BOUNDARY_MAX){
        output = BOUNDARY_MAX;
    }
    else if(output < BOUNDARY_MIN) {
        output = BOUNDARY_MIN;
    }
    pid->output = output;
    
    pid->last_input = pid->input;
}
