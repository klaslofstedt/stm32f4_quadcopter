// inspo: https://github.com/Lauszus/LaunchPadFlightController/blob/master/src/AltitudeHold.c
#include "altitude.h"
#include "barometer.h"
#include "ultrasonic.h"
#include "laser.h"
#include "lidar.h"
#include "imu.h"
#include "uart.h"
#include "filter.h"
#include "gpio.h"
#include "board.h"

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#include "task.h"

#include <math.h>
//#include "tiny_ekf.h"

#define LOOP_TIME_MS 25
#define M_PI 3.14159265358979323846
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

static float altitude_remove_angle_contribution(float estimate, float x, float y);


//xSemaphoreHandle altitude_sem = NULL;
xQueueHandle altitude_queue = 0;

static imu_data_t imu;
static altitude_data_t altitude;

//static void altitude_update(ekf_t* ekf, altitude_data_t* alt);
//static void altitude_set(ekf_t* ekf);
//static void altitude_read_offset(void);

barometer_data_t barometer = {
    .altitude_max = 50000, // 500 m above ground
    .altitude_min = 3000,   // 30 m above ground? what if i want to land lower than initial height?
    .offset = 0
};

laser_data_t laser = {
    .range_max = 150,
    .range_min = 1
};

lidar_data_t lidar = {
    .range_max = 4000,
    .range_min = 100
};


/*ekf_t ekf_altitude;

static void altitude_update(ekf_t* ekf, altitude_data_t* alt)//float u_in, float dt_in)
{ 
double u = (double)alt->acc_z;
double dt = (double)alt->dt / 1000;
// x = Fx + Bu
ekf->fx[0] = (u * pow(dt, 2)/2) + (dt * ekf->x[1]) + ekf->x[0];
ekf->fx[1] = u * dt + ekf->x[1];
ekf->fx[2] = ekf->x[2];
ekf->fx[3] = ekf->x[3];

//ekf->hx = H*ekf->fx;
ekf->hx[0] = ekf->fx[0] + ekf->fx[2];
ekf->hx[1] = ekf->fx[0];
ekf->hx[2] = ekf->fx[0] + ekf->fx[3];

alt->altitude_cm = (float)ekf->fx[0];
alt->rate_cm_s = (float)ekf->fx[1];
        }

static void altitude_set(ekf_t* ekf)
{
// Important (process noise, how characteristics change over time). Small
ekf->Q[0][0] = 0.001; // height 0.3
ekf->Q[1][1] = 0.001; // speed  0.5

// Not as important
ekf->P[0][0] = 0.1;    //0.1
ekf->P[1][1] = 0.1;    //0.1
ekf->P[2][2] = 10000;
ekf->P[3][3] = 10000;

// Important! Needs better method to update this as it needs to be changed runtime
ekf->R[0][0] = 1;
ekf->R[1][1] = 0.025; // if echo
ekf->R[2][2] = 10000; // if no satellites

ekf->x[0] = 0;
ekf->x[1] = 0;
ekf->x[2] = 100;
ekf->x[3] = 100;

ekf->F[0][0] = 1;
ekf->F[1][1] = 1;
ekf->F[2][2] = 1;
ekf->F[3][3] = 1;
ekf->F[0][1] = LOOP_TIME_MS / 1000; // is this right row/column?

ekf->H[0][0] = 1;
ekf->H[0][2] = 1;
ekf->H[1][0] = 1;
ekf->H[2][0] = 1;
ekf->H[2][3] = 1;
        }

static void altitude_read_offset(void)
{
delay_ms(10000);
uint16_t i;
for(i = 0; i < SAMPLES; i++){
altitude.acc_offset += (imu_read_acc_z() / SAMPLES);
delay_ms(1);

//uart_printf("Acc offset: %.5f\n\r", altitude.acc_offset);
            } 
        }*/
static float altitude_remove_angle_contribution(float estimate, float x, float y)
{
    double x_temp = (double)(estimate * sin((double)(x * M_PI / 180)));
    double y_temp = (double)(estimate * sin((double)(y * M_PI / 180)));
    double tot_temp = sqrt(pow((double)estimate, 2) - pow(x_temp, 2) - pow(y_temp, 2));
    
    return (float)tot_temp;
}


void altitude_task(void *pvParameters)
{
    uart_printf("Altitude task\n\r");
    
    UBaseType_t stack_size_alt;
    stack_size_alt = uxTaskGetStackHighWaterMark(NULL);
    
    //barometer_init(&barometer);
    //lidar_init(&lidar);
    laser_init(&laser);
    laser_read_average(&laser, 50);
    barometer.offset = laser.range_avg;
    
    altitude_queue = xQueueCreate(1, sizeof(altitude_data_t));
    
    //ekf_init(&ekf_altitude, Nsta, Mobs);
    //altitude_set(&ekf_altitude);
    TickType_t wake_time = xTaskGetTickCount();
    TickType_t last_wake_time = 0;
    const TickType_t delay = LOOP_TIME_MS; // 25 ms = 40 hz
    while(1)
    { 
        vTaskDelayUntil(&wake_time, delay / portTICK_PERIOD_MS); // 40Hz
        if(xQueueReceive(imu_altitude_queue, &imu, 0)){
            GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_ALT_TASK_PIN);
            // Calc dt
            altitude.dt = ((wake_time - last_wake_time) / portTICK_PERIOD_MS);   
            last_wake_time = wake_time;
            
            laser_read(&laser);
            //lidar_read(&lidar);
            //barometer_read(&barometer, imu.acc_z);
            
            // If range is too low for lidar, use only laser
            if(laser.range_cm >= laser.range_min && laser.range_cm <= lidar.range_min){
                altitude.altitude_cm = altitude_remove_angle_contribution(laser.range_cm, imu_read_dmp_roll(), imu_read_dmp_pitch());
                altitude.rate_cm_s = (altitude.altitude_cm - altitude.altitude_cm_last) / (float)altitude.dt;
                
                altitude.sensor_index = 1;
            }
            // If range is between range and lidar, use both sensors
            else if(laser.range_cm > lidar.range_min && lidar.range_cm < laser.range_max){
                // Constrain in case distance was not previously set and then calculate value in the range [0,1]
                float damping = (float)(constrain(altitude.altitude_cm, lidar.range_min, laser.range_max) - lidar.range_min) / (laser.range_max - lidar.range_min); 
                float range = filter_transition(laser.range_cm, lidar.range_cm, damping);
                
                altitude.altitude_cm = altitude_remove_angle_contribution(range, imu_read_dmp_roll(), imu_read_dmp_pitch());
                altitude.rate_cm_s = (altitude.altitude_cm - altitude.altitude_cm_last) / (float)altitude.dt;
                
                altitude.sensor_index = 2;
            }
            // If range is too high for laser and too low for barometer, use only lidar
            else if(lidar.range_cm > lidar.range_min && lidar.range_cm < barometer.altitude_min){
                altitude.altitude_cm = altitude_remove_angle_contribution(lidar.range_cm, imu_read_dmp_roll(), imu_read_dmp_pitch());
                altitude.rate_cm_s = (altitude.altitude_cm - altitude.altitude_cm_last) / (float)altitude.dt;
                
                altitude.sensor_index = 3;
            }
            // If range is between lidar and barometer, use both sensors
            else if(lidar.range_cm < lidar.range_max && lidar.range_cm > barometer.altitude_min){
                float lidar_height = altitude_remove_angle_contribution(lidar.range_cm, imu_read_dmp_roll(), imu_read_dmp_pitch());
                
                float damping = (float)(constrain(altitude.altitude_cm, barometer.altitude_min, lidar.range_max) - barometer.altitude_min / (lidar.range_max - barometer.altitude_min)); 
                altitude.altitude_cm = filter_transition(lidar.range_cm, barometer.altitude_cm, damping);
                
                altitude.sensor_index = 4;
            }
            // If only barometer is present, use it nevertheless the lower height limit
            else if(barometer.altitude_cm > barometer.altitude_min && barometer.altitude_cm < barometer.altitude_max){
                altitude.altitude_cm = barometer.altitude_cm;
                altitude.rate_cm_s = barometer.rate_cm_s;
                altitude.sensor_index = 5;
            }
            
            else{
                // TODO: use accelerometer?
                //uart_printf("Altitude measurement failed\n\r");
                altitude.sensor_index = 0;
            }
            //float temp_altitude = altitude.altitude_cm;
            //float temp_rate = altitude.rate_cm_s;
            
            // These damping values are just stolen
            float temp1 = altitude.altitude_cm; // remove
            
            altitude.altitude_cm = filter_lowpass(altitude.altitude_cm, altitude.altitude_cm_last, 0.9f); //0.995f
            //uart_printf("$ %d %d %d;", (int16_t)(10*altitude.altitude_cm), (int16_t)(10*temp1), 100);
            //uart_printf("raw: %.4f ", altitude.rate_cm_s);
            altitude.rate_cm_s = filter_lowpass(altitude.rate_cm_s, altitude.rate_cm_s_last, 0.9f); // 0.995f
            //uart_printf(" filter: %.4f\n\r", altitude.rate_cm_s);
            //uart_printf("$ %d %d %d;", 100, (int16_t)(10), (int16_t)1000*altitude.rate_cm_s);
            altitude.altitude_cm_last = altitude.altitude_cm;
            altitude.rate_cm_s_last = altitude.rate_cm_s;
            
            
            
            //uart_printf(" range_true : %.4f", range_true);
            //altitude.acc_z = imu.acc_z;
            //altitude.altitude_cm = range_true;
            
            
            GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_ALT_TASK_PIN);
            xQueueOverwrite(altitude_queue, &altitude);
            //}
            
            
            // Remove additional contribution due to roll & pitch angles
            // h_estimate = sqrt(h_true^2 + roll^2 + pitch^2) gives:
            
            
            
            // plot
            //uart_printf(" $ %d %d;", (int16_t)(10*laser.range_cm), (int16_t)(-1000*altitude.acc_z - altitude.laser_offset));
            
            
            
            
            //uart_printf(" laser: %.2f ", laser.range_cm - altitude.laser_offset);
            //uart_printf(" mbar: %.4f", barometer.mbar);
            //uart_printf(" temp: %.4f", barometer.temp_c);
            //uart_printf(" altitude: %.4f", barometer.altitude_m);
            //uart_printf(" acc_z: %.4f", altitude.acc_z);
            //ultrasonic_read(&ultrasonic);
            //gps_read(&gps);
            
            //double z[3];
            //z[0] = 0;//barometer.altitude_cm;
            //z[1] = (double)laser.range_cm - altitude.laser_offset; // or lidar //ultrasonic.distance_cm;
            //z[2] = 0; // gps?
            
            
            //altitude_update(&ekf_altitude, &altitude); 
            //ekf_step(&ekf_altitude, z); // baro, ultrasonic, GPS?
            
            //uart_printf(" altitude: %.3f ", altitude.altitude_cm);
            //uart_printf(" speed: %.4f ", altitude.rate_cm_s);
            
            
            stack_size_alt = uxTaskGetStackHighWaterMark(NULL);
            altitude.stack_size = stack_size_alt;
        }
        else{
            //uart_printf("No altitude IMU data in queue\n\r");
        }
    }
}
