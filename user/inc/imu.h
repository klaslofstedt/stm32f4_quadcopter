#ifndef IMU_H
#define IMU_H

// FreeRTOS kernel includes
#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "semphr.h"
#include "queue.h"


typedef struct{
	float dmp_roll;
	float dmp_pitch;
	float dmp_yaw;
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
    float acc_x; // y?
    float acc_y; // x??
    float acc_z;
    unsigned long dt;
    UBaseType_t stack_size;
} imu_data_t;

//extern xSemaphoreHandle gyro_new;
extern xSemaphoreHandle imu_done;
extern xQueueHandle imu_data;

void imu_task(void *pvParameters);
void gyro_data_ready_cb(void);

#endif

