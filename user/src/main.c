// C includes
#include "stdio.h"
// Hardware includes
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
// FreeRTOS kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
// User includes
#include "main.h"
#include "pwm.h"
#include "imu.h"
#include "hardware.h"
#include "esc.h"
#include "pid.h"
#include "printf2.h"
#include "joystick.h"
#include "altitude.h"


static void main_task(void *pvParameters);
static void telemetry_task(void *pvParameters);

// 168000000=168Mhz
extern uint32_t SystemCoreClock; 

UBaseType_t stack_size_main;


static imu_data_t imu; // _g or _g?
//static altitude_data_t altitude;

static float thrust = 0; // _g or _g?
static float toggle = 0;



void main_task(void *pvParameters)
{    
    esc_init(&esc1);
    esc_init(&esc2);
    esc_init(&esc3);
    esc_init(&esc4);
    //send a semaphore that it's clear to start acc_measurements ?
    
    float last_yaw = 0;
    stack_size_main = uxTaskGetStackHighWaterMark( NULL );
    while(1){
        if(xSemaphoreTake(imu_attitude_sem, portMAX_DELAY) == pdTRUE){
            if(!xQueueReceive(imu_attitude_queue, &imu, 1000)){ // 1000 ms?
                printf2("No IMU data in queue\n\r");
            }
         
            // Read setpoints --------------------------------------------------
            pitch.setpoint = joystick_get_setpoint(&joystick_pitch_g);
            roll.setpoint = joystick_get_setpoint(&joystick_roll_g);
            yaw.setpoint = joystick_get_setpoint(&joystick_yaw_g);
            //altitude.setpoint = joystick_get_thrust(&joystick_thrust_g);
            thrust = joystick_get_thrust(&joystick_thrust_g);
            toggle = joystick_get_toggle(&joystick_toggle_g);
            
            // Read inputs -----------------------------------------------------
            roll.input = imu.dmp_roll;
            pitch.input = imu.dmp_pitch;
            // TODO: Also read gyro here!!!
            
            // Set yaw to the derivative of IMU reading
            
            yaw.input = (imu.dmp_yaw - last_yaw) / imu.dt;
            last_yaw = imu.dmp_yaw;
            
            pid_altitude.input = altitude_read_cm();
            pid_altitude.rate = altitude_read_speed();
            
            // Poll altitude data
            /*if(xSemaphoreTake(altitude_sem, 0) == pdTRUE){
                if(!xQueueReceive(altitude_data_queue, &altitude, 0)){
                    printf2("No altitude data in queue\n\r");
                }
                //printf2("main altitude\n\r");
                printf2("altitude: %.4f ", altitude.dt);
                printf2("altitude: %.4f\n\r ", altitude.altitude_cm);
            }*/
            
            // Calculate outputs -----------------------------------------------
            pid_calc(&roll, imu.dt);
            pid_calc(&pitch, imu.dt);
            pid_calc(&yaw, imu.dt);
            //pid_calc(&pid_altitude, imu.dt);
            
            // Set outputs ----------------------------------------------------- 
            //  1  front  2
            //  left    right
            //  4   back  3  
            if(toggle > 0.5) { // armed
            esc_set_speed(&esc1, thrust + pitch.output + roll.output + yaw.output);
            esc_set_speed(&esc2, thrust - pitch.output + roll.output - yaw.output);
            esc_set_speed(&esc3, thrust - pitch.output - roll.output + yaw.output);
            esc_set_speed(&esc4, thrust + pitch.output - roll.output - yaw.output);
            }
            else{
                esc_set_speed(&esc1, 0);
                esc_set_speed(&esc2, 0);
                esc_set_speed(&esc3, 0);
                esc_set_speed(&esc4, 0);
            }
            
            //taskYIELD();
        }

        stack_size_main = uxTaskGetStackHighWaterMark( NULL );
    }
}


void telemetry_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = 100; // 1000ms / 5 = 200Hz
    UBaseType_t stack_size_tele;
    stack_size_tele = uxTaskGetStackHighWaterMark( NULL );
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS); // good shit!
        
        //printf2(" pitch: %.3f", joystick_pitch_g.duty_input);
        //printf2(" %.3f", joystick_pitch_g.freq_input);
        //printf2(" roll: %.3f", joystick_roll_g.duty_input);
        //printf2(" %.3f", joystick_roll_g.freq_input);
        //printf2(" yaw: %.3f", joystick_yaw_g.duty_input);
        //printf2(" %.3f", joystick_yaw_g.freq_input);
        //printf2(" thrust: %.3f", joystick_thrust_g.duty_input);
        //printf2(" %.3f", joystick_thrust_g.freq_input);
        //printf2(" toggle: %.3f", joystick_toggle_g.duty_input);
        //printf2(" %.3f", joystick_toggle_g.freq_input);
        
        //printf2(" pitch k_p: %.4f", pitch.k_p);
        //printf2(" pitch k_d: %.4f", pitch.k_d);
        //printf2(" yaw k_d: %.4f", yaw.k_d);
        //printf2("dt: %d", (imu.dt));
        
        //printf2(" roll_set_point: %.3f", roll.setpoint);
        //printf2(" pitch_set_point: %.3f", pitch.setpoint);
        //printf2(" yaw_set_point: %.3f", yaw.setpoint);
        //printf2(" thrust: %.3f", thrust);
        //printf2(" toggle: %.3f", toggle);
        
        //printf2(" x acc: %.3f", imu.acc_x);
        //printf2(" y acc: %.3f", imu.acc_y);
        //printf2(" z acc: %.3f", imu.acc_z);
        
        //printf2(" roll gyro: %.3f", imu.gyro_roll);
        //printf2(" pitch gyro: %.3f", imu.gyro_pitch);
        //printf2(" yaw gyro: %.6f", imu.gyro_yaw); // ideally same thing as yaw.input
        
        //printf2(" roll dmp: %.3f", imu.dmp_roll);
        //printf2(" pitch dmp: %.3f", imu.dmp_pitch);
        //printf2(" yaw dmp ori: %.3f", imu.dmp_yaw);
        //printf2(" yaw dmp rate: %.6f", 1000*yaw.input); // *1000 because dt = 2 and not 0.002
        //printf2(" yaw set_point: %.3f", yaw.setpoint);
        //printf2(" yaw speed: %.3f", 
        //printf2(" yaw: %7.4f", imu.yaw);
        
        
        //printf2(" esc1: %.3f", esc1.speed);
        //printf2(" esc2: %.3f", esc2.speed);
        //printf2(" esc3: %.3f", esc3.speed);
        //printf2(" esc4: %.3f", esc4.speed);
        
        //printf2(" pwm1: %d", pwm_get_duty_cycle(12));
        //printf2(" pwm2: %d", pwm_get_duty_cycle(13));
        //printf2(" pwm3: %d", pwm_get_duty_cycle(14));
        //printf2(" pwm4: %d", pwm_get_duty_cycle(15));
        
        // stack sizes
        //printf2(" main size: %d", stack_size_main);
        //printf2(" tele size: %d", stack_size_tele);
        //printf2(" imu size: %d", imu.stack_size);
        //printf2(" alti size: %d", altitude.stack_size);
        
        // Print to plotter with format ("$%d %d;", data1, data2);
        //printf2(" $%d;", 10*(int16_t)imu.acc_x);
        //printf2(" $%d;", 10*(int16_t)imu.gyro_roll);
        //printf2(" $%d %d %d;", 100*(int16_t)imu.acc_x, 10*(int16_t)imu.gyro_roll, 10*(int16_t)imu.dmp_roll);
        //printf2("\n\r");
        //#endif  
        stack_size_tele = uxTaskGetStackHighWaterMark(NULL);
    }
}



int main(void)
{
    // systick is 100000Hz = 100kHz = 0.1MHz It's not though? 1000Hz i think
    hardware_init();
    
    xTaskCreate(imu_task, (const char *)"imu_task", 300, NULL, 2, NULL);
    
    xTaskCreate(telemetry_task, (const char *)"telemetry_task", 300, NULL, 2, NULL); // telemery_task
    
    xTaskCreate(altitude_task, (const char *)"altitude_task", 400, NULL, 2, NULL);
    
    xTaskCreate(main_task, (const char *)"main_task", 250, NULL, 2, NULL);
    
    //xTaskCreate(barometer_task, (const char *)"barometer_task", 200, NULL, 2, NULL);
    
    //xTaskCreate(ultrasonic_task, (const char *)"ultrasonic_task", 128, NULL, 2, NULL);
    
    //xTaskCreate(gps_task, (const char *)"gps_task", 512, NULL, 2, NULL);
    
    
    
    vTaskStartScheduler();
    
    // should never be reached
    for(;;);
}



