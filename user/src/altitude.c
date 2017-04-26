#include "altitude.h"
#include "barometer.h"
#include "ultrasonic.h"
#include "laser.h"
#include "lidar.h"
#include "imu.h"
#include "printf2.h"
#include "filter.h"

#include "task.h"

#include <math.h>
#include "tiny_ekf.h"

#define LOOP_TIME_MS 250
#define M_PI 3.14159265358979323846
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


xSemaphoreHandle altitude_sem = NULL;
xQueueHandle altitude_data_queue = 0;


static imu_data_t imu;


static float altitude_remove_angle_contribution(float measured, float x, float y);

//static void altitude_update(ekf_t* ekf, altitude_data_t* alt);
//static void altitude_set(ekf_t* ekf);
//static void altitude_read_offset(void);


altitude_data_t altitude;
//imu_data_t imu;


barometer_data_t barometer = {
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
        
        //printf2("Acc offset: %.5f\n\r", altitude.acc_offset);
    } 
}*/
static float altitude_remove_angle_contribution(float measured, float x, float y)
{
    double x_temp = (double)(measured * sin((double)(x * M_PI / 180)));
    double y_temp = (double)(measured * sin((double)(y * M_PI / 180)));
    double tot_temp = sqrt(pow((double)measured, 2) - pow(x_temp, 2) - pow(y_temp, 2));
    
    return (float)tot_temp;
}

float altitude_read_cm(void)
{
    return altitude.altitude_cm;
}

float altitude_read_speed(void)
{
    return altitude.rate_cm_s;
}

void altitude_task(void *pvParameters)
{
    printf2("Altitude task\n\r");
    UBaseType_t stack_size;
    //printf2("init ultrasonic\n\r");
    //ultrasonic_init();
    barometer_init(&barometer);
   
    laser_init(&laser);
    laser_read_average(&laser, 50);
    float range = laser.range_avg;
    barometer.offset = laser.range_avg;
    
    
    //altitude_data = xQueueCreate(1, sizeof(altitude_data_t));
    
    //ekf_init(&ekf_altitude, Nsta, Mobs);
    //altitude_set(&ekf_altitude);
    TickType_t wake_time = xTaskGetTickCount();
    TickType_t last_wake_time = 0;
    const TickType_t delay = LOOP_TIME_MS; // 25 ms = 40 hz
    while(1)
    {
        vTaskDelayUntil(&wake_time, delay / portTICK_PERIOD_MS); // 40Hz
        if(xSemaphoreTake(imu_altitude_sem, 0) == pdTRUE){
            if(!xQueueReceive(imu_altitude_queue, &imu, 0)){
                printf2("No altitude IMU data in queue\n\r");
            }
        }

        //barometer_read(&barometer);
        
        
        laser_read(&laser);
        lidar_read(&lidar);
        
        // If both lidar and laser are working well
        if(laser.range_cm != -1 && lidar.range_cm != -1){
            // If range is too low for lidar, use only laser
            if(laser.range_cm >= laser.range_min && laser.range_cm < lidar.range_min){
                range = laser.range_cm;
            }
            // If range is too high for laser, use only lidar
            else if(lidar.range_cm > laser.range_max && lidar.range_cm <= lidar.range_max){
                range = lidar.range_cm;
            }
            // Else use both sensors
            else{
                // Constrain in case distance was not previously set and then calculate value in the range [0,1]
                float damping = (float)(constrain(range, lidar.range_min, laser.range_max) - lidar.range_min) / (laser.range_max - lidar.range_min); 
                filter_transition(&range, laser.range_cm, lidar.range_cm, damping);
            }
        }
        else if(laser.range_cm != -1){
            if(laser.range_cm >= laser.range_min && laser.range_cm <= laser.range_max){
                range = laser.range_cm;
            }
        }
        else if(lidar.range_cm != -1){
            if(lidar.range_cm >= lidar.range_min && lidar.range_cm <= lidar.range_max){
                range = lidar.range_cm;
            }
        }
        else{
            // range = ????
            printf2("Laser & Lidar measurements failed\n\r");
        }
        printf2(" range: %.4f", range);
        float range_true = altitude_remove_angle_contribution(range, imu_read_dmp_roll(), imu_read_dmp_pitch());
        printf2(" range_true : %.4f", range_true);
        // Remove additional contribution due to roll & pitch angles
        // h_measured = sqrt(h_true^2 + roll^2 + pitch^2) gives:
        
        
        //altitude.dt = (float)(wake_time - last_wake_time);
        //altitude.acc_z = imu_read_acc_z() - altitude.acc_offset;
        
        //last_wake_time = wake_time;
        // plot
        //printf2(" $ %d %d;", (int16_t)(10*laser.range_cm), (int16_t)(-1000*altitude.acc_z - altitude.laser_offset));
        
        
        
        
        //printf2(" laser: %.2f ", laser.range_cm - altitude.laser_offset);
        //printf2(" mbar: %.4f", barometer.mbar);
        //printf2(" temp: %.4f", barometer.temp_c);
        //printf2(" altitude: %.4f", barometer.altitude_m);
        //printf2(" acc_z: %.4f", altitude.acc_z);
        //ultrasonic_read(&ultrasonic);
        //gps_read(&gps);
        
        //double z[3];
        //z[0] = 0;//barometer.altitude_cm;
        //z[1] = (double)laser.range_cm - altitude.laser_offset; // or lidar //ultrasonic.distance_cm;
        //z[2] = 0; // gps?

        
        //altitude_update(&ekf_altitude, &altitude); 
        //ekf_step(&ekf_altitude, z); // baro, ultrasonic, GPS?
        
        //printf2(" altitude: %.3f ", altitude.altitude_cm);
        //printf2(" speed: %.4f ", altitude.rate_cm_s);
        printf2("\n\r");
        
        
        /*
        xQueueOverwrite(altitude_data, &altitude);
        xSemaphoreGive(altitude_sem);
        
        stack_size = uxTaskGetStackHighWaterMark(NULL);
        altitude.stack_size = stack_size;*/
    }
}

