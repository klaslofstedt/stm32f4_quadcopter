// FreeRTOS kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

// user
#include "imu.h"
#include "motors.h"
#include "esc.h"
#include "pid.h"
#include "printf2.h"
// Interrupts
#include "stm32f4xx_it.h"

#define MOTOR1 12
#define MOTOR2 13
#define MOTOR3 14
#define MOTOR4 15

extern volatile float ic1_val, ic2_val, ic3_val, ic4_val;
extern volatile float ic1_duty, ic2_duty, ic3_duty, ic4_duty;
extern volatile float ic1_freq, ic2_freq, ic3_freq, ic4_freq;

static esc_t esc1, esc2, esc3, esc4;
static imu_data_t imu;

static float thrust;


static void motors_set(float roll, float pitch);
static int8_t kinda(float val, float ref, float step);
static void set_setpoint();
static void set_pid();






pid_data_t roll = {
    .input = 0,
    .last_input = 0,
	.set_point = 0,
	.i_term = 0,
	.output = 0,
    /*.k_p = 0.0100,
    .k_i = 0.0,
    .k_d = 0*/
    .k_p = 0.006,
    .k_i = 0.0,
    .k_d = 0.9//1.17
};

pid_data_t pitch = {
    .input = 0,
	.last_input = 0,
	.set_point = 0,
	.i_term = 0,
	.output = 0,
    /*.k_p = 0.0100,
    .k_i = 0.0,
    .k_d = 0*/
    .k_p = 0.006,
    .k_i = 0.0,
    .k_d = 0.9//1.17 // testa 9! (0,0018 *5000 = 9)
};

pid_data_t yaw = {
    .input = 0,
	.last_input = 0,
	.set_point = 0,
	.i_term = 0,
	.output = 0,
    /*
    .k_p = 0.0065 ,
    .k_i = 0.0,
    .k_d = 0.003*/
    .k_p = 0.0060 ,
    .k_i = 0.0,
    .k_d = 0.0018
};

static int8_t kinda(float val, float ref, float step)
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

static void set_pid()
{
    if(kinda(ic1_freq, 55, 1) == 0){
        if(ic1_duty < 7){
            pitch.k_d = pitch.k_d - 0.0005;
            roll.k_d = roll.k_d - 0.0005;
        }
        else if(ic1_duty > 10){
            pitch.k_d = pitch.k_d + 0.0005;
            roll.k_d = roll.k_d + 0.0005;
        }
    }
    if(kinda(ic2_freq, 55, 1) == 0){
        if(ic2_duty < 7){
            pitch.k_p = pitch.k_p - 0.000002;
            roll.k_p = roll.k_p - 0.000002;
        }
        else if(ic2_duty > 10){
            pitch.k_p = pitch.k_p + 0.000002;
            roll.k_p = roll.k_p + 0.000002;
        }
    }
}

static void set_setpoint()
{
    if(kinda(ic1_freq, 55, 1) == 0) {
        if(kinda(ic1_duty, 8.35, 0.15) == -1){
            pitch.set_point = (10 * (8.35 - ic1_duty));
            roll.set_point = (10 * (8.35 - ic1_duty));
        }
        else if(kinda(ic1_duty, 8.35, 0.15) == 1){
            pitch.set_point = -(10 * (ic1_duty - 8.35));
            roll.set_point = -(10 * (ic1_duty - 8.35));
        }
        else{
            pitch.set_point = 0;
            roll.set_point = 0;
        }
    }
    if(pitch.set_point > 15){
        pitch.set_point = 15;
    }
    if(pitch.set_point < -15){
        pitch.set_point = -15;
    }
    if(roll.set_point > 15){
        roll.set_point = 15;
    }
    if(roll.set_point < -15){
        roll.set_point = -15;
    }
}

static void motors_set(float roll, float pitch)
{
    thrust = 0.6; // sänk och ladda batteri
    //((float)USART1_RxByte) /100;
    
    /* 
    1  front  2
	left    right
	4   back  3 */
    
    esc2.speed = thrust - pitch + roll; // - yawValue;
    esc1.speed = thrust + pitch + roll; // + yawValue;
    esc4.speed = thrust + pitch - roll; // - yawValue;
    esc3.speed = thrust - pitch - roll; // + yawValue;
    /*esc2.speed = thrust - yaw;
    esc1.speed = thrust + yaw;
    esc4.speed = thrust - yaw;
    esc3.speed = thrust + yaw;*/
    
    esc_check_bondaries(&esc1);
    esc_check_bondaries(&esc2);
    esc_check_bondaries(&esc3);
    esc_check_bondaries(&esc4);
    
    esc_set_speed(&esc1);
    esc_set_speed(&esc2);
    esc_set_speed(&esc3);
    esc_set_speed(&esc4);
    //printf2("speed\n\r");
}

void motors_task(void *pvParameters)
{    
    esc_init(&esc1, MOTOR1);
    esc_init(&esc2, MOTOR2);
    esc_init(&esc3, MOTOR3);
    esc_init(&esc4, MOTOR4);
    
    while(1){
        if(xSemaphoreTake(imu_done, portMAX_DELAY) == pdTRUE){
            if(!xQueueReceive(imu_data, &imu, 1000)){ // 1000 ms?
                printf2("No IMU data in queue\n\r");
            }
            // Set P and D parameters
            //set_pid();
            set_setpoint();
            
            roll.input = imu.dmp_roll;
            pitch.input = imu.dmp_pitch;
            yaw.input = imu.dmp_yaw;
            
            pid_calc(&roll, imu.dt);
            pid_calc(&pitch, imu.dt);
            //pid_calc(&yaw, imu.dt);
            //get_tick_count(&ms2);
#ifdef DEBUG
            printf2("dt: %d\n\r", imu.dt);
#endif
            
            motors_set(roll.output, pitch.output);
            
            // ta emot köad data från IMU
            
            // FÅR MAN INTE UT RATE AV DMP??
            //taskYIELD();
        }
    }
}


void print_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = 50; // 1000ms / 5 = 200Hz
    
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS); // good shit!
        
        //#ifdef DEBUG
        printf2(" duty1: %.3f", ic1_duty);
        printf2(" freq1: %f", ic1_freq);
        printf2(" duty2: %.3f", ic2_duty);
        printf2(" freq2: %f", ic2_freq);
        printf2(" pitch k_p: %.4f", pitch.k_p);
        printf2(" pitch k_d: %.4f", pitch.k_d);
        //printf2("dt: %d\n\r", (imu.dt));
        //printf2(" roll_set_point: %.3f", roll.set_point);
        //printf2(" pitch_set_point: %.3f", pitch.set_point);
        printf2(" roll: %.3f", imu.dmp_roll);
        printf2(" pitch: %.3f\n\r", imu.dmp_pitch);
        //printf2(" yaw: %7.4f\n\r", imu.yaw);*/
        //#endif  
    }
}