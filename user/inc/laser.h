#ifndef LASER_H
#define LASER_H

#include <stdint.h>

typedef struct{
    uint16_t range_mm; // m? // float?
    float range_cm;
    float range_avg;
    float range_max;
    float range_min;
} laser_data_t;

void laser_init(laser_data_t *in);
void laser_read(laser_data_t *in);
void laser_read_average(laser_data_t *in, uint32_t samples);

#endif