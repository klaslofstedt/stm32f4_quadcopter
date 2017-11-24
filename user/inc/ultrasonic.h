#ifndef ULTRASONIC_H
#define ULTRASONIC_H

// FreeRTOS kernel includes
#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
#include "semphr.h"
#include "queue.h"

typedef enum
{
    SRF02_DEFAULT_ADDRESS =     0xE0,
    SRF02_ASSIGNED_ADDRESS =    0xE2
} srf02_address_t;

typedef enum
{
    SRF02_REAL_RANGING_MODE_IN =    0x50,
    SRF02_REAL_RANGING_MODE_CM =    0x51,
    SRF02_REAL_RANGING_MODE_MS =    0x52,
    SRF02_FAKE_RANGING_MODE_IN =    0x56,
    SRF02_FAKE_RANGING_MODE_CM =    0x57,
    SRF02_FAKE_RANGING_MODE_MS =    0x58,
    SRF02_TRANSMIT_BURST =          0x5C,
    SRF02_AUTOOTUNE_RESTART =       0x60,
    SRF02_CHANGE_I2C_ADR_1 =        0xA0,
    SRF02_CHANGE_I2C_ADR_2 =        0xAA,
    SRF02_CHANGE_I2C_ADR_3 =        0xA5
} srf02_cmd_t;

typedef enum
{
    SRF02_SOFTWARE_REVISION =   0x00,
    SRF02_COMMAND =             0x00,
    SRF02_UNUSED =              0x01,
    SRF02_RANGE_HIGH =          0x02,
    SRF02_RANGE_LOW =           0x03,
    SRF02_AUTOTUNE_MIN =        0x04,
    SRF02_AUTOTUNE_MAX =        0x05
} srf02_reg_t;


typedef struct{
	uint8_t distance_cm;
    uint8_t address;
} ultrasonic_data_t;

enum ultrasonic_status { IDLE, TRIGGER, TRIGGER_READY, ECHO, ECHO_READY };

//void ultrasonic_task(void *pvParameters);
void ultrasonic_read(ultrasonic_data_t* ultrasonic);
void ultrasonic_init();

#endif