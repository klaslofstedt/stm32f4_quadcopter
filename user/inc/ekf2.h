#ifndef EKF2_H
#define EKF2_H


typedef struct{
    // static inputs
    float Q_angle;
    float Q_bias;
    float R_measure;
    // variables
    float P[2][2];
    float bias;
    // inputs
    float newAngle;
    float newRate;
    float dt; 
    // outputs 
    float angle;
    float rate;
} ekf2_data_t;

void ekf2_calc(ekf2_data_t *ekf2);



#endif
