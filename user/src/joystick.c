#include "joystick.h"
// Interrupts
#include "stm32f4xx_it.h"

static int8_t joystick_accuracy(float val, float ref, float step);
static void joystick_filter(joystick_data_t* in);


static void joystick_filter(joystick_data_t* in)
{
    in->avg[0] = in->duty_input;
    // Calculate a moving average. Every term must add up to (2520*in->avg[n] / 2520), hence the odd last term.
    in->avg[0] = in->avg[0]/4 + in->avg[1]/5 + in->avg[2]/6 + in->avg[3]/7 + in->avg[4]/8 + in->avg[5]/9 + (11*in->avg[6]/2520);
    // Move the average
    in->avg[6] = in->avg[5];
    in->avg[5] = in->avg[4];
    in->avg[4] = in->avg[3];
    in->avg[3] = in->avg[2];
    in->avg[2] = in->avg[1];
    in->avg[1] = in->avg[0];
    
    in->duty_input = in->avg[0];
}

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

float joystick_read_setpoint(joystick_data_t* in)
{
    in->freq_input = isr_read_freq(in->pin_num);
    in->duty_input = isr_read_duty(in->pin_num);
    joystick_filter(in);
    
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

float joystick_read_thrust(joystick_data_t* in)
{
    in->freq_input = isr_read_freq(in->pin_num);
    in->duty_input = isr_read_duty(in->pin_num);
    joystick_filter(in);
    
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

float joystick_read_toggle(joystick_data_t* in)
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