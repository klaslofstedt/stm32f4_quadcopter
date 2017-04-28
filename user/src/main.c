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

#define X_CONFIG 1


static void main_task(void *pvParameters);
static void telemetry_task(void *pvParameters); 
uint8_t armed(void);

UBaseType_t stack_size_main;


static imu_data_t imu; // _g or _g?
static altitude_data_t altitude;

static float thrust = 0; // _g or _g?
static float toggle = 0;

uint8_t armed(void)
{
    toggle = joystick_read_toggle(&joystick_toggle);
    if(toggle > 0.5) {
        return 1;
    }
    else{
        return 0;
    }
}

void main_task(void *pvParameters)
{    
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
                printf2("No IMU data in queue\n\r");
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
            //printf2(" $ %d;", (int32_t)(20000+100*pid_yaw.setpoint));
            
            //data1 = (int32_t)(2000*pid_yaw.setpoint);
            //data1 = (int32_t)(data1 - (0.5 * data1) + 0.5 * data2);
            //data1 = (int32_t)(data1/4 + data2/5 + data3/6 + data4/7 + data5/8 + data6/9 + (11*data7/2520));// + data6/64 + data7/128 + data8/256 + data9/512 + data10/1024);

            //data10 = data9;
            //data9 = data8;
            //data8 = data7;
            /*data7 = data6;
            data6 = data5;
            data5 = data4;
            data4 = data3;
            data3 = data2;
            data2 = data1;*/
            
            

            printf2("$ %d", (int32_t)(2000*pid_yaw.setpoint));
            printf2(" %d", 15000);
            printf2(" %d;", 20000);
            
            pid_yaw.input = imu.gyro_yaw;
            pid_calc(&pid_yaw, imu.dt);

            // Build altitude pid object ---------------------------------------
            // Poll queue for altitude data (~25 ms = 40 Hz interval)
            if(xQueueReceive(altitude_queue, &altitude, 0)){
                //uart_print("Found altitude data in queue\n\r"); // expected
                pid_altitude.setpoint = joystick_read_thrust(&joystick_thrust);
                pid_altitude.input = altitude.altitude_cm;
                pid_altitude.rate = altitude.rate_cm_s;
                //pid_calc(&pid_altitude, altitude.dt);
                
                // Fake line!
                pid_altitude.output = joystick_read_thrust(&joystick_thrust);
            }

            // Set outputs ----------------------------------------------------- 
            if(armed()) { // picture below is wrong
#if X_CONFIG
                //  1  front  2
                //  left    right
                //  4   back  3 
                //printf2("ARM x\n\r");
                esc_set_speed(&esc1, pid_altitude.output + pid_pitch.output + pid_roll.output + pid_yaw.output);
                esc_set_speed(&esc2, pid_altitude.output - pid_pitch.output + pid_roll.output - pid_yaw.output);
                esc_set_speed(&esc3, pid_altitude.output - pid_pitch.output - pid_roll.output + pid_yaw.output);
                esc_set_speed(&esc4, pid_altitude.output + pid_pitch.output - pid_roll.output - pid_yaw.output);
#elif PLUS_CONFIG
                //      1 front
                //  4 left  2 right
                //      3 back
                //printf2("ARM +n\r");
                esc_set_speed(&esc1, pid_altitude.output + pid_pitch.output + pid_yaw.output);
                esc_set_speed(&esc2, pid_altitude.output + pid_roll.output - pid_yaw.output);
                esc_set_speed(&esc3, pid_altitude.output - pid_pitch.output + pid_yaw.output);
                esc_set_speed(&esc4, pid_altitude.output - pid_roll.output - pid_yaw.output);
#endif
            }
            else{
                //printf2("Not armed\n\r");
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
    const TickType_t frequency = 200; // 1000ms / 5 = 200Hz
    UBaseType_t stack_size_tele;
    stack_size_tele = uxTaskGetStackHighWaterMark( NULL );
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS); // good shit!
        
        //printf2(" pitch: %.3f", joystick_pitch_g.duty_input);
        //printf2(" %.3f", joystick_pitch_g.freq_input);
        //printf2(" roll: %.3f", joystick_roll_g.duty_input);
        //printf2(" %.3f", joystick_roll_g.freq_input);
        //printf2(" yaw: %.3f", joystick_yaw.duty_input);
        //printf2(" %.3f", joystick_yaw.freq_input);
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
        //printf2(" yaw set_point: %.3f", pid_yaw.setpoint);
        //printf2(" yaw speed: %.3f", 
        //printf2(" yaw: %7.4f", imu.yaw);
        
        //printf2(" altitude_cm: %.3f", altitude.altitude_cm);
        //printf2(" altitude_acc: %.3f", altitude.acc_z);
        //printf2(" altitude_dt: %d", altitude.dt);
        
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



