#include "laser.h"
#include "vl53l0x.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "uart.h"

VL53L0X_RangingMeasurementData_t vl53l0x_measurement;

void laser_init(laser_data_t *in)
{
    VL53L0X_init();
}

void laser_read(laser_data_t *in)
{
    rangingTest(&vl53l0x_measurement);
    if (vl53l0x_measurement.RangeStatus != 4) {  // phase failures have incorrect data
        //printf2("Distance (mm): %d\n\r", vl53l0x_measurement.RangeMilliMeter);
        in->range_mm = vl53l0x_measurement.RangeMilliMeter;
        in->range_cm = (float)in->range_mm / 10;
    } else {
        //printf2(" out of range ");
        in->range_cm = -1;
        in->range_mm = -1;
    }
}

void laser_read_average(laser_data_t *in, uint32_t samples)
{
    uint8_t i;
    float average = 0;
    for(i = 0; i < samples; i++){
        laser_read(in);
        average += in->range_cm / samples;
    }
    in->range_avg = average;
}
