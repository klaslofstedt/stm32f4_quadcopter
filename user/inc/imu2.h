#ifndef IMU2_H
#define IMU2_H

// FreeRTOS kernel includes
#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "semphr.h"
#include "queue.h"


typedef struct{
    // Usable data
	float angle_x;
	float angle_y;
	float angle_z;
    float rate_x;
    float rate_y;
    float rate_z;
    // Raw data
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x; // y?
    float acc_y; // x??
    float acc_z;
    // other
    float acc_z_average;
    unsigned long dt;
    UBaseType_t stack_size;
} imu_data_t;

extern xSemaphoreHandle imu_attitude_sem;
extern xSemaphoreHandle imu_altitude_sem;
extern xQueueHandle imu_attitude_queue;
extern xQueueHandle imu_altitude_queue;

void imu2_task(void *pvParameters);
float imu2_read_acc_z(void);
float imu2_read_dmp_roll(void);
float imu2_read_dmp_pitch(void);

#endif
