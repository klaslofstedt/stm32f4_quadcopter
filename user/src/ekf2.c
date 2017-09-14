#include "ekf2.h"



// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
void ekf2_calc(ekf2_data_t *ekf2)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    ekf2->rate = ekf2->newRate - ekf2->bias;
    ekf2->angle += ekf2->dt * ekf2->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    ekf2->P[0][0] += ekf2->dt * (ekf2->dt*ekf2->P[1][1] - ekf2->P[0][1] - ekf2->P[1][0] + ekf2->Q_angle);
    ekf2->P[0][1] -= ekf2->dt * ekf2->P[1][1];
    ekf2->P[1][0] -= ekf2->dt * ekf2->P[1][1];
    ekf2->P[1][1] += ekf2->Q_bias * ekf2->dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = ekf2->P[0][0] + ekf2->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = ekf2->P[0][0] / S;
    K[1] = ekf2->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = ekf2->newAngle - ekf2->angle; // Angle difference
    /* Step 6 */
    ekf2->angle += K[0] * y;
    ekf2->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = ekf2->P[0][0];
    float P01_temp = ekf2->P[0][1];

    ekf2->P[0][0] -= K[0] * P00_temp;
    ekf2->P[0][1] -= K[0] * P01_temp;
    ekf2->P[1][0] -= K[1] * P00_temp;
    ekf2->P[1][1] -= K[1] * P01_temp;
};