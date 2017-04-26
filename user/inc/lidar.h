#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

typedef struct{
    float range_cm; // m?
    float range_max;
    float range_min;
} lidar_data_t;

void lidar_init(lidar_data_t *in);
void lidar_read(lidar_data_t *in);

#endif