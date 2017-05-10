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
#include "uart.h"
#include "joystick.h"
#include "altitude.h"
#include "arm.h"

#define X_CONFIG 1


static void main_task(void *pvParameters);
static void telemetry_task(void *pvParameters); 

UBaseType_t stack_size_main;

static imu_data_t imu;
static altitude_data_t altitude;


void main_task(void *pvParameters)
{
    delay_ms(1000);
    esc_init(&esc1);
    esc_init(&esc2);
    esc_init(&esc3);
    esc_init(&esc4);
    
    // Not needed?
    stack_size_main = uxTaskGetStackHighWaterMark(NULL);
    
    while(1){
        // Wait forever for semaphore from imu task.
        // This sem sets the frequency of the main loop (~5 ms = 200 Hz interval)
        if(xSemaphoreTake(imu_attitude_sem, portMAX_DELAY) == pdTRUE){
            if(!xQueueReceive(imu_attitude_queue, &imu, 1000)){ // 1000 ms?
                uart_printf("No IMU data in queue\n\r");
            }
            
            // Build pitch pid object ------------------------------------------
            pid_pitch.setpoint = joystick_read_setpoint(&joystick_pitch);
            pid_pitch.input = imu.dmp_pitch;
            pid_pitch.rate = imu.gyro_pitch;
            pid_calc(&pid_pitch, imu.dt);
            
            // Build roll pid object -------------------------------------------
            pid_roll.setpoint = joystick_read_setpoint(&joystick_roll);
            pid_roll.input = imu.dmp_roll;
            pid_roll.rate = imu.gyro_roll;
            pid_calc(&pid_roll, imu.dt);
            
            // Build yaw pid object --------------------------------------------
            pid_yaw.setpoint = joystick_read_setpoint(&joystick_yaw);
            
            /*printf2("$ %d", (int32_t)(2000*pid_yaw.setpoint));
            printf2(" %d", 15000);
            printf2(" %d;", 20000);*/
            
            pid_yaw.input = imu.gyro_yaw;
            pid_calc(&pid_yaw, imu.dt);
            
            // Build altitude pid object ---------------------------------------
            // Poll queue for altitude data (~25 ms = 40 Hz interval)
            if(xQueueReceive(altitude_queue, &altitude, 0)){
                //uart_print("Found altitude data in queue\n\r"); // expected
                pid_altitude.setpoint = 50;//joystick_read_thrust(&joystick_thrust);
                pid_altitude.input = altitude.altitude_cm;
                pid_altitude.rate = altitude.rate_cm_s;
                pid_calc(&pid_altitude, altitude.dt);
                
                // Fake line!
                //pid_altitude.output = joystick_read_thrust(&joystick_thrust);
            }
            
            // Set outputs ----------------------------------------------------- 
            if(arm()) { // picture below is wrong
#if X_CONFIG
                //  1  front  2
                //  left    right
                //  4   back  3 
                esc_set_speed(&esc1, pid_altitude.output + pid_pitch.output + pid_roll.output + pid_yaw.output);
                esc_set_speed(&esc2, pid_altitude.output - pid_pitch.output + pid_roll.output - pid_yaw.output);
                esc_set_speed(&esc3, pid_altitude.output - pid_pitch.output - pid_roll.output + pid_yaw.output);
                esc_set_speed(&esc4, pid_altitude.output + pid_pitch.output - pid_roll.output - pid_yaw.output);
#elif PLUS_CONFIG
                //      1 front
                //  4 left  2 right
                //      3 back
                esc_set_speed(&esc1, pid_altitude.output + pid_pitch.output + pid_yaw.output);
                esc_set_speed(&esc2, pid_altitude.output + pid_roll.output - pid_yaw.output);
                esc_set_speed(&esc3, pid_altitude.output - pid_pitch.output + pid_yaw.output);
                esc_set_speed(&esc4, pid_altitude.output - pid_roll.output - pid_yaw.output);
#endif
            }
            else{
                esc_set_speed(&esc1, 0);
                esc_set_speed(&esc2, 0);
                esc_set_speed(&esc3, 0);
                esc_set_speed(&esc4, 0);
            }
        }
        // Read the size of this task in order to refine assigned stack size
        stack_size_main = uxTaskGetStackHighWaterMark(NULL);
    }
}


void telemetry_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = 200; // every 200ms = 5Hz
    UBaseType_t stack_size_tele;
    stack_size_tele = uxTaskGetStackHighWaterMark( NULL );
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS); // good shit!
#if X_CONFIG
        //uart_printf("config x\n\r");
#elif PLUS_CONFIG
        //uart_printf("config +\n\r");
#endif
        //uart_printf("armed: %d", arm());
        //uart_printf(" pitch: %.3f", joystick_pitch_g.duty_input);
        //uart_printf(" %.3f", joystick_pitch_g.freq_input);
        //uart_printf(" roll: %.3f", joystick_roll_g.duty_input);
        //uart_printf(" %.3f", joystick_roll_g.freq_input);
        //uart_printf(" yaw: %.3f", joystick_yaw.duty_input);
        //uart_printf(" %.3f", joystick_yaw.freq_input);
        //uart_printf(" thrust: %.3f", joystick_thrust.duty_input);
        //uart_printf(" %.3f", joystick_thrust_g.freq_input);
        //uart_printf(" toggle: %.3f", joystick_toggle_g.duty_input);
        //uart_printf(" %.3f", joystick_toggle_g.freq_input);
        
        //uart_printf(" pitch k_p: %.4f", pitch.k_p);
        //uart_printf(" pitch k_d: %.4f", pitch.k_d);
        //uart_printf(" yaw k_d: %.4f", yaw.k_d);
        //uart_printf("dt: %d", (imu.dt));
        
        //uart_printf(" roll_set_point: %.3f", roll.setpoint);
        //uart_printf(" pitch_set_point: %.3f", pitch.setpoint);
        //uart_printf(" yaw_set_point: %.3f", yaw.setpoint);
        //uart_printf(" thrust: %.3f", thrust);
        //uart_printf(" toggle: %.3f", toggle);
        
        //uart_printf(" x acc: %.3f", imu.acc_x);
        //uart_printf(" y acc: %.3f", imu.acc_y);
        //uart_printf(" z acc: %.3f", imu.acc_z);
        
        //uart_printf(" roll gyro: %.3f", imu.gyro_roll);
        //uart_printf(" pitch gyro: %.3f", imu.gyro_pitch);
        //uart_printf(" yaw gyro: %.6f", imu.gyro_yaw); // ideally same thing as yaw.input
        
        //uart_printf(" roll dmp: %.3f", imu.dmp_roll);
        //uart_printf(" pitch dmp: %.3f", imu.dmp_pitch);
        //uart_printf(" yaw dmp ori: %.3f", imu.dmp_yaw);
        
        //uart_printf(" yaw dmp rate: %.6f", 1000*yaw.input); // *1000 because dt = 2 and not 0.002
        //uart_printf(" yaw set_point: %.3f", pid_yaw.setpoint);
        //uart_printf(" yaw speed: %.3f", 
        //uart_printf(" yaw: %7.4f", imu.yaw);
        
        //uart_printf(" altitude_cm: %.3f", altitude.altitude_cm);
        //uart_printf(" altitude_acc: %.3f", altitude.acc_z);
        //uart_printf(" altitude_dt: %d", altitude.dt);
        //uart_printf(" altitude_index: %d", altitude.sensor_index);
        
        //uart_printf(" esc1: %.3f", esc1.speed);
        //uart_printf(" esc2: %.3f", esc2.speed);
        //uart_printf(" esc3: %.3f", esc3.speed);
        //uart_printf(" esc4: %.3f", esc4.speed);
        
        //uart_printf(" pwm1: %d", pwm_get_duty_cycle(12));
        //uart_printf(" pwm2: %d", pwm_get_duty_cycle(13));
        //uart_printf(" pwm3: %d", pwm_get_duty_cycle(14));
        //uart_printf(" pwm4: %d", pwm_get_duty_cycle(15));
        
        // stack sizes
        //uart_printf(" main size: %d", stack_size_main);
        //uart_printf(" tele size: %d", stack_size_tele);
        //uart_printf(" imu size: %d", imu.stack_size);
        //uart_printf(" alti size: %d", altitude.stack_size);
        
        // Print to plotter with format ("$%d %d;", data1, data2);
        //uart_printf(" $%d;", 10*(int16_t)imu.acc_x);
        //uart_printf(" $%d;", 10*(int16_t)imu.gyro_roll);
        //uart_printf(" $%d %d %d;", 100*(int16_t)imu.acc_x, 10*(int16_t)imu.gyro_roll, 10*(int16_t)imu.dmp_roll);
        //uart_printf("\n\r"); 
        stack_size_tele = uxTaskGetStackHighWaterMark(NULL);
    }
}



int main(void)
{
    // systick is 100000Hz = 100kHz = 0.1MHz It's not though? 1000Hz i think
    hardware_init();
    
    // Reads the imu and pass data on interrupt to main_task
    xTaskCreate(imu_task, (const char *)"imu_task", 300, NULL, 2, NULL);
    // Prints debug data
    xTaskCreate(telemetry_task, (const char *)"telemetry_task", 300, NULL, 2, NULL);
    // Read several height sensors and pass altitude hold data to main_task
    xTaskCreate(altitude_task, (const char *)"altitude_task", 400, NULL, 2, NULL);
    // Reads joystick and process all data before setting new output to ESCs
    xTaskCreate(main_task, (const char *)"main_task", 250, NULL, 2, NULL);

    vTaskStartScheduler();
    
    // should never be reached
    for(;;);
}



