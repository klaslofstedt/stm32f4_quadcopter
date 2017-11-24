// C includes
#include "stdio.h"
#include <math.h> // pi and sinf
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
#include "adc.h"
#include "imu.h"
#include "hardware.h"
#include "esc.h"
#include "pid.h"
#include "uart.h"
#include "joystick.h"
#include "altitude.h"
#include "arm.h"
#include "board.h"
#include "motordriver.h"
#include "debug.h"

#define M_PI acos(-1.0) // pi


static void main_task(void *pvParameters);
static void telemetry_task(void *pvParameters); 

static UBaseType_t stack_size_main;     // Main stack size obj
static imu_data_t imu;                  // imu data obj
static altitude_data_t altitude;        // alt data obj



void delay_us(uint32_t delay) //?
{
    delay = delay*168;
	while (delay != 0){
        delay--;
    }
}


void main_task(void *pvParameters)
{
    stack_size_main = uxTaskGetStackHighWaterMark(NULL); // Not needed?
    
    while(1){
        //output1 = sin_lookup2[phase1];
        //output2 = sin_lookup2[phase2];
        //output3 = sin_lookup2[phase3];
        //output1 = (uint16_t)(65535 * sin_lookup(phase1));
        //output2 = (uint16_t)(65535 * sin_lookup(phase2));
        //output3 = (uint16_t)(65535 * sin_lookup(phase3));
        
        /*phase1 += omega;
        phase2 += omega;
        phase3 += omega;
        if(phase1 > 47){
        phase1 = 0;
    }
        if(phase2 > 47){
        phase2 = 0;
    }        
        if(phase3 > 47){
        phase3 = 0;
    }*/
        /*if(phase1 > 2 * M_PI){
        phase1 = 0;
    }
        if(phase2 > 2 * M_PI){
        phase2 = 0;
    }        
        if(phase3 > 2 * M_PI){
        phase3 = 0;
    }*/
        
        //pwm_set_duty_cycle(12, output1); // 16-bit
        //pwm_set_duty_cycle(13, output2);
        //pwm_set_duty_cycle(14, output3);
        uint16_t adc_ch0 = adc_read(ADC_Channel_0);
        uint16_t adc_ch1 = adc_read(ADC_Channel_1);
        delay_ms(100);
        uart_printf(" ch0: %d", adc_ch0);
        uart_printf(" ch1: %d\n\r", adc_ch1);
        //delay_us(20);
        //if(xQueueReceive(imu_attitude_queue, &imu, 200)){ // Sample 200Hz
        //vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS);
        
        //GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_MAIN_TASK_PIN);
        
        
        // Build pitch PID object ------------------------------------------
        //pid_pitch.setpoint = joystick_read_setpoint(&joystick_pitch);
        //pid_pitch.input = imu.dmp_pitch;
        //pid_pitch.rate = imu.gyro_pitch;
        // Build roll PID object -------------------------------------------
        //pid_roll.setpoint = joystick_read_setpoint(&joystick_roll);
        //pid_roll.input = imu.dmp_roll;
        //pid_roll.rate = imu.gyro_roll;
        // Build yaw PID object --------------------------------------------
        //pid_yaw.setpoint = joystick_read_setpoint(&joystick_yaw);
        //pid_yaw.input = imu.gyro_yaw;
        
        
        // Calculate PID outputs
        //pid_calc(&pid_pitch, imu.dt);
        //pid_calc(&pid_roll, imu.dt);
        //pid_calc(&pid_yaw, imu.dt);      
        
        // Read the size of this task in order to refine assigned stack size
        stack_size_main = uxTaskGetStackHighWaterMark(NULL);
        //GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_MAIN_TASK_PIN);
        //}
        //else{
        //    uart_printf("No IMU queue\n\r");
        //}
    } 
}




void telemetry_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = 200; // every 200ms = 5Hz
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
        uart_printf(" thrust: %.3f", joystick_thrust.duty_input);
        //uart_printf(" %.3f", joystick_thrust_g.freq_input);
        //uart_printf(" toggle: %.3f", joystick_toggle_g.duty_input);
        //uart_printf(" %.3f", joystick_toggle_g.freq_input);
        
        //uart_printf(" pit k_p: %.5f", pid_pitch.k_p);
        //uart_printf(" pit k_i: %.7f", pid_pitch.k_i);
        //uart_printf(" pit k_d: %.5f", pid_pitch.k_d);
        //uart_printf(" i_term: %.4f", pid_pitch.i_term);
        //uart_printf(" rol k_p: %.5f", pid_roll.k_p);
        //uart_printf(" rol k_i: %.7f", pid_roll.k_i);
        //uart_printf(" rol k_d: %.5f", pid_roll.k_d);
        //uart_printf(" i_term: %.4f", pid_pitch.i_term);
        
        //uart_printf(" yaw k_p: %.4f", pid_yaw.k_p);
        //uart_printf(" temp1: %.4f", temp1);
        //uart_printf(" temp2: %.4f", temp2);
        //uart_printf(" k_p: %.4f", pid_altitude.k_p);
        //uart_printf(" k_d: %.4f", pid_altitude.k_d);
        //uart_printf(" k_p: %.4f", pid_thrust.k_p);
        //uart_printf(" k_d: %.4f", pid_thrust.k_d);
        
        //uart_printf("dt: %d", (imu.dt));
        
        //uart_printf(" roll_set_point: %.3f", roll.setpoint);
        //uart_printf(" pitch_set_point: %.3f", pitch.setpoint);
        //uart_printf(" yaw_set_point: %.3f", yaw.setpoint);
        //uart_printf(" alt_setpoint: %.3f", pid_altitude.setpoint);
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
        
        uart_printf(" roll dmp: %.3f", imu.dmp_roll);
        //uart_printf(" pitch dmp: %.3f", imu.dmp_pitch);
        //uart_printf(" yaw dmp ori: %.3f", imu.dmp_yaw);
        
        //uart_printf(" yaw dmp rate: %.6f", 1000*yaw.input); // *1000 because dt = 2 and not 0.002
        //uart_printf(" yaw set_point: %.3f", pid_yaw.setpoint);
        //uart_printf(" yaw speed: %.3f", 
        //uart_printf(" yaw: %7.4f", imu.yaw);
        
        //uart_printf(" alt_cm: %.3f", altitude.altitude_cm);
        //uart_printf(" rate: %.3f", altitude.rate_cm_s);
        //uart_printf(" altitude_acc: %.3f", altitude.acc_z);
        //uart_printf(" altitude_dt: %d", altitude.dt);
        //uart_printf(" alt_index: %d", altitude.sensor_index);
        
        
        uart_printf(" thrust: %.3f", pid_altitude.output);
        //uart_printf(" thrust: %.3f", pid_thrust.output);
        
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



/*
  I/O pins used to control the motor driver.
  This for the STM32F4 Discovery board.
  Channel 1 IN use channel 1/2/4 of TIM5 on PA0/1/3.
  Channel 2 IN use channel 1/3/4 of TIM3 on PC6/8/9.
  Channel 3 IN use channel 1/3/4 of TIM2 on PA15/PA2/PB11.
  EN use pins PE7-15
*/

int main(void)
{
    // Setup STM32 system (clock, PLL and Flash configuration)
    SystemInit();
    // Update the system clock variable (is this already done?)
    SystemCoreClockUpdate();
    // This on is set to 4 according to FreeRTOS spec. Do not change.
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    // Peripherals and stuff
    //uart_init();
    //debug_init();
    //i2c2_init(); // imu
    //adc_init();
    motordriver_init();
    
    uint32_t m_delay = 1000; // set from pid
    uint32_t loop1 = 400;
    uint32_t loop2 = 400;
    uint32_t loop3 = 400;
    
    uint32_t i = 0;//, count1, count2, count3;
    
    
    while (1)
    {
        if(i >= m_delay){
            i = 0;
        }
        i++;
        //for (i= 0; i < m_delay; ++i)
        //{
            //uint32_t j = (i + m_delay/3) % m_delay;
            //uint32_t k = (i + 2*m_delay/3) % m_delay;
            motordriver_set1(i, m_delay);
            //set_channels_2(0.5f*(1.0f+sinf((2.0f*M_PI)*((float)i/(float)m_delay))),
            //               0.5f*(1.0f+sinf((2.0f*M_PI)*((float)j/(float)m_delay))),
            //               0.5f*(1.0f+sinf((2.0f*M_PI)*((float)k/(float)m_delay))));
            //set_channels_3(0.5f*(1.0f+sinf((2.0f*M_PI)*((float)i/(float)m_delay))),
            //               0.5f*(1.0f+sinf((2.0f*M_PI)*((float)j/(float)m_delay))),
            //               0.5f*(1.0f+sinf((2.0f*M_PI)*((float)k/(float)m_delay))));
            
        //}
    }
    
    // Reads the imu and pass data on interrupt to main_task
    //xTaskCreate(imu_task, (const char *)"imu_task", 350, NULL, 4, NULL);
    // Prints debug data
    //xTaskCreate(telemetry_task, (const char *)"telemetry_task", 300, NULL, 1, NULL);
    // Read several height sensors and pass altitude hold data to main_task
    //xTaskCreate(altitude_task, (const char *)"altitude_task", 400, NULL, 2, NULL);
    // Init ESC, reads joystick and process all data before setting new output to ESCs
    //xTaskCreate(main_task, (const char *)"main_task", 300, NULL, 3, NULL);
    
    //vTaskStartScheduler();
    
    // should never be reached
    
    for(;;);
}



