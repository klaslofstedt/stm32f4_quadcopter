#include "pid.h"

#define BOUNDARY_MIN -1.0f
#define BOUNDARY_MAX 1.0f

// Might wanna have a better error handling when using delta input as rate:
// https://github.com/Lauszus/LaunchPadFlightController/blob/fddbe4eb9303ea4301a714585d7383a2de275d80/src/PID.c

void pid_calc(pid_data_t* pid, unsigned long dt)
{
    /*uart_printf(" input: %.4f", pid->input);
    uart_printf(" rate: %.4f", pid->rate);
    uart_printf(" setpoint: %.4f", pid->setpoint);*/
    // These are used for ease to read
    float p_term, i_term, d_term, error, output;
    
    // Calculate input rate with derivation of position instead from EKF output
    // *** IMPORTANT *** Does it need low pass-filter?                        
    //pid->rate = (pid->input - pid->last_input) / (float)dt;
    
    // Calculate error between current and desired position
    error = pid->setpoint - pid->input;
    //uart_printf(" error: %.4f", error);
    // Calculate the P contribution
    p_term = pid->k_p * error;
    //uart_printf(" p_term: %.4f", p_term);
    // Calculate the I contribution
    pid->i_term += (float)(pid->k_i * (float)dt * error);
    if(pid->i_term > pid->boundary_max){
        pid->i_term = pid->boundary_max;
    }
    else if(pid->i_term < pid->boundary_min) {
        i_term = pid->boundary_min;
    }
    i_term = pid->i_term;
    //uart_printf(" i_term: %.4f", i_term);
    // Calculate the D contribution
    d_term = pid->k_d * pid->rate;
    //uart_printf(" d_term: %.4f", d_term);
    //Calculate output
    output = p_term + i_term - d_term;
    //uart_printf(" output_raw: %.4f", output);
    // Check boundaries
    if(output > pid->boundary_max){
        output = pid->boundary_max;
    }
    else if(output < pid->boundary_min) {
        output = pid->boundary_min;
    }
    pid->output = output;
    //uart_printf(" output: %.4f", output);
    //uart_printf("\n\r"); 
    pid->last_input = pid->input;
}
