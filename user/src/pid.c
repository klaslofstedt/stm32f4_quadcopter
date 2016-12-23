#include "pid.h"

#define BOUNDARY_MIN -1.0f
#define BOUNDARY_MAX 1.0f

void pid_calc(pid_data_t* pid, unsigned long dt)
{
    dt = 3333;
    // These are used for ease to read
    float p_term, i_term, d_term, error, output, input_rate;
    
    // Calculate input rate from old and new state instead from EKF output
    input_rate = (pid->input - pid->last_input) / (float)dt;
    
    //printf2("input %0.4f", pid->input);
    //printf2("input_rate %0.4f", input_rate);
    
    // Calculate error between current and desired position
    error = pid->set_point - pid->input;
    
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
    d_term = pid->k_d * input_rate;
    //printf2("d_term %0.4f\n\r", d_term);
    
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
