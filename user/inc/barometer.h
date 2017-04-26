#ifndef BAROMETER_H
#define BAROMETER_H

#include "FreeRTOS.h"
#include <stdint.h>

typedef enum
{
    MS5803_ADDRESS_HIGH = 0x76, // CSB to VCC
    MS5803_ADDRESS_LOW  = 0x77  // CSB to GND
} ms5803_address_t;

typedef enum
{
    MS5803_CMD_RESET = 0x1E,
    MS5803_CMD_PROM  = 0xA0,
    MS5803_CMD_READ  = 0x00
} ms5803_cmd_t;

typedef enum
{
    MS5803_D2_256  = 0x50,
    MS5803_D2_512  = 0x52,
    MS5803_D2_1024 = 0x54,
    MS5803_D2_2048 = 0x56,
    MS5803_D2_4096 = 0x58
} ms5803_temperature_t;

typedef enum
{
    MS5803_D1_256  = 0x40,
    MS5803_D1_512  = 0x42,
    MS5803_D1_1024 = 0x44,
    MS5803_D1_2048 = 0x46,
    MS5803_D1_4096 = 0x48
} ms5803_pressure_t;

typedef struct{
    float mbar;
    float temp_c;
    float altitude_m;
    float altitude_cm;
    float altitude_ground_m;
    float altitude_ground_cm;
    float offset;
} barometer_data_t;

//void barometer_task(void *pvParameters);
void barometer_init();
void barometer_read(barometer_data_t* barometer);

#endif
