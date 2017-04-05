/*#include "kalman.h"
#include <math.h>
#include "tiny_ekf.h"

static void kalman_update(ekf_t * ekf, double u, float dt)
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

static void kalman_set(ekf_t* ekf)
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
    
}

void kalman_calc()
{
    // called from e.g. motors.c??? before the loop (just once)
    ekf_t ekf;
    ekf_init(&ekf, Nsta, Mobs);
    kalman_set(&ekf);
    
    double u = 0;
    double z[3];
    
    z[0] = 0;
    z[1] = 0;
    z[2] = 0;
    
    // is this where x = Ax + Bu is happening?? Are A and B constants-ish?
    kalman_update(&ekf, u, 50); // SV_Pos = accelerometer?
    //
    ekf_step(&ekf, z); // SV_Rho = baro, ultrasonic, GPS?
}*/
    