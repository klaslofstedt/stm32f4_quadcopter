#include "joystick.h"
// Interrupts
#include "stm32f4xx_it.h"

static int8_t joystick_accuracy(float val, float ref, float step);

static int8_t joystick_accuracy(float val, float ref, float step)
{
    if(val < (ref + step) && val > (ref - step)){
        return 0;
    }
    else if(val <= (ref - step)){
        return -1;
    }
    else{
        // when val >= (ref + 1)
        return 1;
    }
}

/*static void set_pid()
{
    if(joystick_accuracy(ic1_freq, 55, 1) == 0){
        if(ic1_duty < 7){
            pitch.k_d = pitch.k_d - 0.0005;
            roll.k_d = roll.k_d - 0.0005;
        }
        else if(ic1_duty > 10){
            pitch.k_d = pitch.k_d + 0.0005;
            roll.k_d = roll.k_d + 0.0005;
        }
    }
    if(joystick_accuracy(ic2_freq, 55, 1) == 0){
        if(ic2_duty < 7){
            pitch.k_p = pitch.k_p - 0.000002;
            roll.k_p = roll.k_p - 0.000002;
        }
        else if(ic2_duty > 10){
            pitch.k_p = pitch.k_p + 0.000002;
            roll.k_p = roll.k_p + 0.000002;
        }
    }
}*/

float joystick_get_setpoint(joystick_data_t* in)
{
    in->freq_input = isr_read_freq(in->pin_num);
    in->duty_input = isr_read_duty(in->pin_num);
    
    float output = 0;
    if(joystick_accuracy(in->freq_input, in->freq_desired, in->freq_accuracy) == 0) {
        if(joystick_accuracy(in->duty_input, in->duty_center, in->duty_thresh) == -1){
            output = (in->scalefactor * (in->duty_center - in->duty_input));
        }
        else if(joystick_accuracy(in->duty_input, in->duty_center, in->duty_thresh) == 1){
            output = -(in->scalefactor * (in->duty_input - in->duty_center));
        }
        else{
            output = 0;
        }
    }
    if(output > in->output_limit){
        output = in->output_limit;
    }
    if(output < -in->output_limit){
        output = -in->output_limit;
    }
    return output;
}

float joystick_get_thrust(joystick_data_t* in)
{
    in->freq_input = isr_read_freq(in->pin_num);
    in->duty_input = isr_read_duty(in->pin_num);
    
    float output = 0;
    if(joystick_accuracy(in->freq_input, in->freq_desired, in->freq_accuracy) == 0) {
        if(joystick_accuracy(in->duty_input, in->duty_max, in->duty_thresh) == -1){
            output = -(in->scalefactor * (in->duty_input - in->duty_max));
        }
        else{
            output = 0;
        }
    }
    if(output > in->output_limit){
        output = in->output_limit;
    }
    return output;
}

float joystick_get_toggle(joystick_data_t* in)
{
    in->freq_input = isr_read_freq(in->pin_num);
    in->duty_input = isr_read_duty(in->pin_num);
    
    float output = 0;
    if(joystick_accuracy(in->freq_input, in->freq_desired, in->freq_accuracy) == 0) {
        if(in->duty_input > in->duty_center){
            output = 1;
        }
        else{
            output = 0;
        }
    }
    return output;
}