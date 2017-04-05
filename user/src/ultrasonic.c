#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

/* Demo program include files. */
#include "i2c_1.h"
#include "printf2.h"
#include "freertos_time.h"
#include "ultrasonic.h"


volatile enum ultrasonic_status ultrasonic_status_g = IDLE;

void ultrasonic_read(ultrasonic_data_t* ultrasonic)
{
    delay_ms(1000);
    printf2("1\n\r");
    I2C_TransmitReg(SRF02_ASSIGNED_ADDRESS, SRF02_COMMAND, SRF02_REAL_RANGING_MODE_CM);
    printf2("2\n\r");
    delay_ms(70);
    I2C_TransmitByte(SRF02_ASSIGNED_ADDRESS, SRF02_RANGE_HIGH);
    delay_ms(70);
    printf2("3\n\r");
    uint8_t temp[2];
    I2C_Receive(SRF02_ASSIGNED_ADDRESS, temp, 2);
    printf2("4\n\r");
    ultrasonic->distance_cm = temp[0] << 8 | temp[1];
}

void ultrasonic_init()
{    
    printf2("a\n\r");
    delay_ms(10);
    I2C_TransmitReg(SRF02_DEFAULT_ADDRESS, SRF02_COMMAND, SRF02_CHANGE_I2C_ADR_1);
    delay_ms(10);
    printf2("b\n\r");
    I2C_TransmitReg(SRF02_DEFAULT_ADDRESS, SRF02_COMMAND, SRF02_CHANGE_I2C_ADR_2);
    delay_ms(10);
    printf2("c\n\r");
    I2C_TransmitReg(SRF02_DEFAULT_ADDRESS, SRF02_COMMAND, SRF02_CHANGE_I2C_ADR_3);
    delay_ms(10);
    printf2("d\n\r");
    I2C_TransmitReg(SRF02_DEFAULT_ADDRESS, SRF02_COMMAND, SRF02_ASSIGNED_ADDRESS);
    printf2("e\n\r");
    delay_ms(10);
}


/*void ultrasonic_task(void *pvParameters)
{
    ultrasonic_init();
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = 50; // 1000ms / 5 = 200Hz
    float distance_cm = 10;
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS); // good shit!
        
        printf2(" distance_cm: %.3f", distance_cm);

        printf2("\n\r"); 
    }
}*/