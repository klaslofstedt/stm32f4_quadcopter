#ifndef ALTITUDE_H
#define ALTITUDE_H

// FreeRTOS kernel includes
#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "semphr.h"
#include "queue.h"

/* states */
#define Nsta 4
/* observables */
#define Mobs 3

extern xSemaphoreHandle altitude_done;
extern xQueueHandle altitude_data;

// This is only kalman filter struct for ONE tiny_ekf object!! this case altitude
typedef struct {
    // [row][column]
    int n;          // number of state values
    int m;          // number of observables

    double x[Nsta];         // state vector

    double P[Nsta][Nsta];   // prediction error covariance 
    double Q[Nsta][Nsta];   // process noise covariance 
    double R[Mobs][Mobs];   // measurement error covariance 

    double G[Nsta][Mobs];   // Kalman gain; a.k.a. K 

    double F[Nsta][Nsta];   // Jacobian of process model 
    double H[Mobs][Nsta];   // Jacobian of measurement model 

    double Ht[Nsta][Mobs];  // transpose of measurement Jacobian 
    double Ft[Nsta][Nsta];  // transpose of process Jacobian 
    double Pp[Nsta][Nsta];  // P, post-prediction, pre-update 

    double fx[Nsta];        // output of user defined f() state-transition function 
    double hx[Mobs];        // output of user defined h() measurement function 

    // temporary storage 
    double tmp0[Nsta][Nsta];
    double tmp1[Nsta][Mobs];
    double tmp2[Mobs][Nsta];
    double tmp3[Mobs][Mobs];
    double tmp4[Mobs][Mobs];
    double tmp5[Mobs]; 

} ekf_t; 

typedef struct {
    float altitude_cm;
    float dt;
} altitude_data_t;

void altitude_task(void *pvParameters);

#endif