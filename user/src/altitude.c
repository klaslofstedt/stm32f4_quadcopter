#include "altitude.h"
#include "barometer.h"
#include "ultrasonic.h"
#include "printf2.h"

#include "task.h"

#include <math.h>
#include "tiny_ekf.h"

static void altitude_update(ekf_t* ekf, double u, float dt);
static void altitude_set(ekf_t* ekf);

xSemaphoreHandle altitude_done = NULL;
xQueueHandle altitude_data = 0;


barometer_data_t barometer = {
    .mbar = 0,
    .temp_c = 0,
    .altitude_m = 0,
    .stack_size = 0
};

ultrasonic_data_t ultrasonic = {
    .address = 0xE2,
    .distance_cm = 0
};

altitude_data_t altitude = {
    .altitude_cm = 0
};

static void altitude_update(ekf_t* ekf, double u, float dt)
{ 
    // x = Fx + Bu
    ekf->fx[0] = (u * pow(dt, 2)/2) + (dt * ekf->x[1]) + ekf->x[0];
    ekf->fx[1] = u * dt + ekf->x[1];
    ekf->fx[2] = ekf->x[2];
    ekf->fx[3] = ekf->x[3];
    
    //ekf->hx = H*ekf->fx;
    ekf->hx[0] = ekf->fx[0] + ekf->fx[2];
    ekf->hx[1] = ekf->fx[0];
    ekf->hx[2] = ekf->fx[0] + ekf->fx[3];
}

static void altitude_set(ekf_t* ekf)
{
    ekf->Q[0][0] = 0.3;
    ekf->Q[1][1] = 0.5;
    
    ekf->P[0][0] = 0.1;
    ekf->P[1][1] = 0.1;
    ekf->P[2][2] = 10000;
    ekf->P[3][3] = 10000;
    
    // Needs better method to update this as it needs to be changed runtime
    ekf->R[0][0] = 1;
    ekf->R[1][1] = 0.25; // if echo
    ekf->R[2][2] = 10000; // if no satellites

    ekf->x[0] = 0;
    ekf->x[1] = 0;
    ekf->x[2] = 100;
    ekf->x[3] = 100;
    
    int dt = 50;
    ekf->F[0][0] = 1;
    ekf->F[1][1] = 1;
    ekf->F[2][2] = 1;
    ekf->F[3][3] = 1;
    ekf->F[0][1] = dt; // is this right row/column?
    
    ekf->H[0][0] = 1;
    ekf->H[0][2] = 1;
    ekf->H[1][0] = 1;
    ekf->H[2][0] = 1;
    ekf->H[2][3] = 1;
    
    /*ekf->Q[0][0] = 0.001;
    ekf->Q[1][1] = 0.003;
    
    ekf->P[0][0] = 1;
    ekf->P[1][1] = 1;
    
    // Needs better method to update this as it needs to be changed runtime
    ekf->R[0][0] = 0.03;
    ekf->R[1][1] = 0.02; // if echo

    ekf->H[0][0] = 1;
    ekf->H[0][1] = 0;
    
    ekf->x[0] = 0;
    ekf->x[1] = 0;
    ekf->x[2] = 100;
    ekf->x[3] = 100;
    
    int dt = 1;
    ekf->F[0][0] = 1;
    ekf->F[1][1] = 1;
    ekf->F[2][2] = 1;
    ekf->F[3][3] = 1;*/
}


void altitude_task(void *pvParameters)
{
    delay_ms(10000);
    UBaseType_t stack_size;
    printf2("init ultrasonic\n\r");
    ultrasonic_init();
    //barometer_init();
    
    //altitude_data = xQueueCreate(1, sizeof(altitude_data_t));
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t delay = 500; // 1000ms / 5 = 200Hz
    
    float distance_cm = 10;
    
    // called from e.g. motors.c??? before the loop (just once)
    //ekf_t ekf;
    //ekf_init(&ekf, Nsta, Mobs);
    //altitude_set(&ekf);*/
    
    
    
    
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, delay / portTICK_PERIOD_MS); // good shit!
        
        //barometer_read(&barometer);
        
        ultrasonic_read(&ultrasonic);
        //gps_read(&gps);
        
        /*double z[3];
        z[0] = barometer.altitude_m;
        z[1] = ultrasonic.distance_cm;
        z[2] = 0;*/
        
        double u = 0;
        
        //printf2(" altitude: %.4f", barometer.altitude_m);
        printf2(" distance_cm: %d", ultrasonic.distance_cm);
        printf2("\n\r"); 
        
        // find out a way to read accelerometer!!!!
        /*altitude_update(&ekf, u, 50); 
        ekf_step(&ekf, z); // baro, ultrasonic, GPS?
        
        if(!xQueueSend(altitude_data, &altitude, 1000)){
            printf2("xQueueSend altitude failed\n\r");
        }
        xSemaphoreGive(altitude_done);
        
        stack_size = uxTaskGetStackHighWaterMark(NULL);
        barometer.stack_size = stack_size;*/
    }
}

