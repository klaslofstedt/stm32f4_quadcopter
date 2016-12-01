#ifndef IMU_H
#define IMU_H

// FreeRTOS kernel includes
#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "semphr.h"
#include "queue.h"


typedef struct{
	float roll;
	float pitch;
	float yaw;
    unsigned long dt;
} imu_data_t;

//extern xSemaphoreHandle gyro_new;
extern xSemaphoreHandle imu_done;
extern xQueueHandle imu_data;

void imu_task(void *pvParameters);
void gyro_data_ready_cb(void);

#endif

