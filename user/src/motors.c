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

#define MOTOR1 12
#define MOTOR2 13
#define MOTOR3 14
#define MOTOR4 15

static void motors_set(float roll, float pitch);

esc_t esc1, esc2, esc3, esc4;

pid_data_t roll = {
    .input = 0,
    .last_input = 0,
	.set_point = 0,
	.i_term = 0,
	.output = 0,
    /*.k_p = 0.0100,
    .k_i = 0.0,
    .k_d = 0*/
    .k_p = 0.0065,
    .k_i = 0.0,
    .k_d = 9.0
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
    .k_p = 0.0065,
    .k_i = 0.0,
    .k_d = 9.0 // testa 9! (0,0018 *5000 = 9)
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

// den oscillerar lite
static void motors_set(float roll, float pitch)
{
    float thrust = 0.45; // sänk och ladda batteri
    //((float)USART1_RxByte) /100;
    
    esc2.speed = thrust - pitch + roll; // - yawValue;
    esc1.speed = thrust + pitch + roll; // + yawValue;
    esc4.speed = thrust + pitch - roll; // - yawValue;
    esc3.speed = thrust - pitch - roll; // + yawValue;
    
    esc_check_bondaries(&esc1);
    esc_check_bondaries(&esc2);
    esc_check_bondaries(&esc3);
    esc_check_bondaries(&esc4);
    
    esc_set_speed(&esc1);
    esc_set_speed(&esc2);
    esc_set_speed(&esc3);
    esc_set_speed(&esc4);
}

void motors_task(void *pvParameters)
{
    imu_data_t imu;
    esc_init(&esc1, MOTOR1);
    esc_init(&esc2, MOTOR2);
    esc_init(&esc3, MOTOR3);
    esc_init(&esc4, MOTOR4);
    
    
    while(1){
        if(xSemaphoreTake(imu_done, portMAX_DELAY) == pdTRUE){
            if(!xQueueReceive(imu_data, &imu, 1000)){ // 1000 ms?
                printf2("No IMU data in queue\n\r");
            }
            /*printf2("dt: %d\n\r", (imu.dt));
            printf2(" roll: %7.4f", imu.roll);
            printf2(" pitch: %7.4f", imu.pitch);
            printf2(" yaw: %7.4f\n\r", imu.yaw);*/
            
            roll.input = imu.roll;
            pitch.input = imu.pitch;
            yaw.input = imu.yaw;
            
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