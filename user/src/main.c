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
#include "board.h"

#define X_CONFIG 1


static void main_task(void *pvParameters);
static void telemetry_task(void *pvParameters); 

UBaseType_t stack_size_main;

static imu_data_t imu;
static altitude_data_t altitude;
float temp1 = 0, temp2 = 0, temp3 = 0;
unsigned long tick1 = 0, tick2 = 0;

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
        // This queue sets the frequency of the main loop (~5 ms = 200 Hz interval)
        if(xQueueReceive(imu_attitude_queue, &imu, 15)){
            GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_MAIN_TASK_PIN);
            
            // Build pitch pid object ------------------------------------------
            
            temp1 = joystick_read_setpoint(&joystick_pitch);
            if(temp1 < -14){
                //pid_roll.k_i = pid_roll.k_i - 0.00001;
                //pid_pitch.k_i = pid_pitch.k_i - 0.00001;
                pid_roll.k_p = pid_roll.k_p - 0.00001;
                pid_pitch.k_p = pid_pitch.k_p - 0.00001;
                //pid_altitude.k_p = pid_altitude.k_p - 0.0001;
            }
            else if(temp1 > 14){
                //pid_roll.k_i = pid_roll.k_i + 0.00001;
                //pid_pitch.k_i = pid_pitch.k_i + 0.00001;
                pid_roll.k_p = pid_roll.k_p + 0.00001;
                pid_pitch.k_p = pid_pitch.k_p + 0.00001;
                //pid_altitude.k_p = pid_altitude.k_p + 0.0001;
            }
            
            pid_pitch.setpoint = 0;//joystick_read_setpoint(&joystick_pitch);
            pid_pitch.input = imu.dmp_pitch;
            pid_pitch.rate = imu.gyro_pitch;
            
            
            // Build roll pid object -------------------------------------------
            
            temp2 = joystick_read_setpoint(&joystick_roll);
            if(temp2 < -14){
                pid_roll.k_d = pid_roll.k_d - 0.000001;
                pid_pitch.k_d = pid_pitch.k_d - 0.000001;
                //pid_altitude.k_d = pid_altitude.k_d - 0.01;
            }
            else if(temp2 > 14){
                pid_roll.k_d = pid_roll.k_d + 0.000001;
                pid_pitch.k_d = pid_pitch.k_d + 0.000001;
                //pid_altitude.k_d = pid_altitude.k_d + 0.01;
            }
            pid_roll.setpoint = 0;//joystick_read_setpoint(&joystick_roll);
            pid_roll.input = imu.dmp_roll;
            pid_roll.rate = imu.gyro_roll;
            
            
            // Build yaw pid object --------------------------------------------
            pid_yaw.setpoint = 0;            
            /*printf2("$ %d", (int32_t)(2000*pid_yaw.setpoint));*/
            temp3 = joystick_read_setpoint(&joystick_yaw);
            if(temp3 < -14){
                //pid_roll.k_i = pid_roll.k_i - 0.0000001;
                //pid_pitch.k_i = pid_pitch.k_i - 0.0000001;
                //pid_altitude.k_d = pid_altitude.k_d - 0.01;
            }
            else if(temp3 > 14){
                //pid_roll.k_i = pid_roll.k_i + 0.0000001;
                //pid_pitch.k_i = pid_pitch.k_i + 0.0000001;
                //pid_altitude.k_d = pid_altitude.k_d + 0.01;
            }
            pid_yaw.input = imu.gyro_yaw;
            
            
            
            // Build altitude pid object ---------------------------------------
            // Poll queue for altitude data (~25 ms = 40 Hz interval)
            if(xQueueReceive(altitude_queue, &altitude, 0)){
                //uart_print("Found altitude data in queue\n\r"); // expected
                pid_altitude.setpoint = (float)(100*joystick_read_thrust(&joystick_thrust));
                pid_altitude.input = altitude.altitude_cm;
                pid_altitude.rate = altitude.rate_cm_s;
                //pid_calc(&pid_altitude, altitude.dt);
                // Fake line!
                //pid_altitude.output = joystick_read_thrust(&joystick_thrust);
            }
            pid_altitude.output = joystick_read_thrust(&joystick_thrust);
            
            
            
            
            
            // Set outputs ----------------------------------------------------- 
            if(arm() && joystick_read_thrust(&joystick_thrust) > 0.001f){ // if arm and thrust joystick
                // Calculate PID
                pid_calc(&pid_pitch, imu.dt);
                pid_calc(&pid_roll, imu.dt);
                pid_calc(&pid_yaw, imu.dt);
                //pid_calc(&pid_altitude, altitude.dt);
                
                //  1  front  2
                //  left    right
                //  4   back  3 
                /*esc_set_speed(&esc1, pid_altitude.output - pid_pitch.output - pid_roll.output - pid_yaw.output);
                esc_set_speed(&esc2, pid_altitude.output + pid_pitch.output - pid_roll.output + pid_yaw.output);
                esc_set_speed(&esc3, pid_altitude.output + pid_pitch.output + pid_roll.output - pid_yaw.output);
                esc_set_speed(&esc4, pid_altitude.output - pid_pitch.output + pid_roll.output + pid_yaw.output);*/
                esc_set_speed(&esc1, pid_altitude.output + pid_roll.output + pid_pitch.output/* + pid_yaw.output*/);
                esc_set_speed(&esc2, pid_altitude.output + pid_roll.output - pid_pitch.output/* - pid_yaw.output*/);
                esc_set_speed(&esc3, pid_altitude.output - pid_roll.output - pid_pitch.output/* + pid_yaw.output*/);
                esc_set_speed(&esc4, pid_altitude.output - pid_roll.output + pid_pitch.output/* - pid_yaw.output*/);
                
            }
            else{
                esc_set_speed(&esc1, 0);
                esc_set_speed(&esc2, 0);
                esc_set_speed(&esc3, 0);
                esc_set_speed(&esc4, 0);
            }
            
            // Read the size of this task in order to refine assigned stack size
            stack_size_main = uxTaskGetStackHighWaterMark(NULL);
            GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_MAIN_TASK_PIN);
        }
        else{
            uart_printf("No IMU data in queue\n\r");
        }
    } 
}


void telemetry_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = 400; // every 200ms = 5Hz
    UBaseType_t stack_size_tele;
    stack_size_tele = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS);
        GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_TEL_TASK_PIN);
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
        
        uart_printf(" pit k_p: %.5f", pid_pitch.k_p);
        //uart_printf(" pit k_i: %.7f", pid_pitch.k_i);
        uart_printf(" pit k_d: %.5f", pid_pitch.k_d);
        //uart_printf(" i_term: %.4f", pid_pitch.i_term);
        //uart_printf(" rol k_p: %.5f", pid_roll.k_p);
        //uart_printf(" rol k_i: %.7f", pid_roll.k_i);
        //uart_printf(" rol k_d: %.5f", pid_roll.k_d);
        //uart_printf(" i_term: %.4f", pid_pitch.i_term);
        
        //uart_printf(" yaw k_d: %.4f", yaw.k_d);
        //uart_printf(" temp1: %.4f", temp1);
        //uart_printf(" temp2: %.4f", temp2);
        //uart_printf(" k_p: %.4f", pid_altitude.k_p);
        //uart_printf(" k_d: %.4f", pid_altitude.k_d);
        
        //uart_printf("dt: %d", (imu.dt));
        
        //uart_printf(" roll_set_point: %.3f", roll.setpoint);
        //uart_printf(" pitch_set_point: %.3f", pitch.setpoint);
        //uart_printf(" yaw_set_point: %.3f", yaw.setpoint);
        //uart_printf(" altitude_setpoint: %.3f", pid_altitude.setpoint);
        //uart_printf(" thrust: %.3f", thrust);
        //uart_printf(" toggle: %.3f", toggle);
        
        //uart_printf(" x acc: %.3f", imu.acc_x);
        //uart_printf(" y acc: %.3f", imu.acc_y);
        //uart_printf(" z acc: %.3f", imu.acc_z);
        
        //uart_printf(" roll gyro: %.3f", imu.gyro_roll);
        //uart_printf(" pitch gyro: %.3f", imu.gyro_pitch);
        //uart_printf(" yaw gyro: %.6f", imu.gyro_yaw); // ideally same thing as yaw.input
        
        //uart_printf(" roll gyro: %.6f", imu.gyro_roll);
        //uart_printf(" pid rate: %.6f", pid_roll.rate);
        //uart_printf(" pid calc: %.6f", pid_roll.rate_calc);
        
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
        
        
        //uart_printf(" joystick_thrust: %.3f", joystick_read_thrust(&joystick_thrust));
        
        
        //uart_printf(" esc1: %.4f", esc1.speed);
        //uart_printf(" esc2: %.4f", esc2.speed);
        //uart_printf(" esc3: %.4f", esc3.speed);
        //uart_printf(" esc4: %.4f", esc4.speed);
        
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
        //uart_printf(" $ %d %d %d;", 100, (int16_t)((1000 * imu.gyro_roll) + 1000), 500);
        uart_printf("\n\r"); 
        
        GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_TEL_TASK_PIN);
        stack_size_tele = uxTaskGetStackHighWaterMark(NULL);
    }
}



int main(void)
{
    // systick is 100000Hz = 100kHz = 0.1MHz It's not though? 1000Hz i think
    hardware_init();
    
    // Reads the imu and pass data on interrupt to main_task
    xTaskCreate(imu_task, (const char *)"imu_task", 350, NULL, 4, NULL);
    // Prints debug data
    xTaskCreate(telemetry_task, (const char *)"telemetry_task", 300, NULL, 1, NULL);
    // Read several height sensors and pass altitude hold data to main_task
    xTaskCreate(altitude_task, (const char *)"altitude_task", 400, NULL, 2, NULL);
    // Init ESC, reads joystick and process all data before setting new output to ESCs
    xTaskCreate(main_task, (const char *)"main_task", 300, NULL, 3, NULL);
    
    vTaskStartScheduler();
    
    // should never be reached
    for(;;);
}



