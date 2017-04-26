#include "freertos_time.h"
#include <stdio.h>
#include "printf2.h"
#include "i2c_1.h"
#include "barometer.h"
#include <math.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

static uint32_t barometer_read_pressure();
static uint32_t barometer_read_temperature();
static uint8_t barometer_validate_crc();
static void barometer_reset(ms5803_cmd_t reset);
static void barometer_read_prom();

uint16_t prom_coefficient[8];

// Some constants used in calculations below
//const uint64_t POW_2_33 = 8589934592ULL; // 2^33 = 8589934592
//const uint64_t POW_2_37 = 137438953472ULL; // 2^37 = 137438953472

void barometer_init()
{
    printf2("Initializing barometer...\n\r");
    barometer_reset(MS5803_CMD_RESET);
    barometer_read_prom();
    if(barometer_validate_crc()){
        printf2("Barometer initialized\n\r");
    }
    else{
        while(1){
            printf2("err: bad CRC");
            delay_ms(1000);
        }
    }
}

static uint32_t barometer_read_pressure()
{
    uint8_t in_buffer[3];
    uint32_t pressure_raw = 0;
    Sensors_I2C1_Write(MS5803_ADDRESS_HIGH, MS5803_D1_4096);
    //I2C_TransmitByte(MS5803_ADDRESS_HIGH, MS5803_D1_4096); // send 2 bytes
    delay_ms(10);
    Sensors_I2C1_ReadRegister(MS5803_ADDRESS_HIGH, MS5803_CMD_READ, 3, in_buffer);
    //I2C_TransmitByte(MS5803_ADDRESS_HIGH, MS5803_CMD_READ); // send 2 bytes
    //I2C_Receive(MS5803_ADDRESS_HIGH, in_buffer, 3); // pointer???
    pressure_raw = ((uint32_t)in_buffer[2]) | ((uint32_t)in_buffer[1] << 8) | ((uint32_t)in_buffer[0] << 16); 
    return pressure_raw;
}

static uint32_t barometer_read_temperature()
{
    uint8_t in_buffer[3];
    uint32_t temperature_raw = 0;
    Sensors_I2C1_Write(MS5803_ADDRESS_HIGH, MS5803_D2_4096);
    //I2C_TransmitByte(MS5803_ADDRESS_HIGH, MS5803_D2_4096); // send 2 bytes
    delay_ms(10);
    Sensors_I2C1_ReadRegister(MS5803_ADDRESS_HIGH, MS5803_CMD_READ, 3, in_buffer);
    
    //I2C_TransmitByte(MS5803_ADDRESS_HIGH, MS5803_D2_4096); // send 2 bytes
    delay_ms(10);
    //I2C_TransmitByte(MS5803_ADDRESS_HIGH, MS5803_CMD_READ); // send 2 bytes
    //I2C_Receive(MS5803_ADDRESS_HIGH, in_buffer, 3);
    temperature_raw = ((uint32_t)in_buffer[2]) | ((uint32_t)in_buffer[1] << 8) | ((uint32_t)in_buffer[0] << 16);
    return temperature_raw;
}

static void barometer_reset(ms5803_cmd_t reset)
{
    //printf2("1!");
    Sensors_I2C1_Write(MS5803_ADDRESS_HIGH, reset);
    delay_ms(1000);
    //printf2("2!");
}

void barometer_read(barometer_data_t* barometer)
{
    int32_t pressure_raw = barometer_read_pressure();
    //D1 = pressure_raw;
    int32_t temperature_raw = barometer_read_temperature();
    //D2 = temperature_raw;
    int32_t temp_calc;
	int32_t pressure_calc;	
	int32_t dt;
    
    
    dt = temperature_raw - ((int32_t)prom_coefficient[5] << 8 );
    // Use integer division to calculate TEMP. It is necessary to cast
    // one of the operands as a signed 64-bit integer (int64_t) so there's no 
    // rollover issues in the numerator.
    temp_calc = (((int64_t)dt * prom_coefficient[6]) >> 23) + 2000;
    
    static int64_t	offset = 0;
    static int64_t	sensitivity  = 0;
    static int64_t	dt2 = 0;
    static int64_t	offset2 = 0;
    static int64_t	sensitivity2 = 0;
    
    if (temp_calc < 2000) {
		// For 14 bar model
		// If temperature is below 20.0C
		dt2 = 3 * (((int64_t)dt * dt) >> 33);
		//dt2 = (int32_t)dt2; // recast as signed 32bit integer
		offset2 = 3 * ((temp_calc-2000) * (temp_calc-2000)) / 2;
		sensitivity2 = 5 * ((temp_calc-2000) * (temp_calc-2000)) / 8;
        
        if (temp_calc < -1500) {
		// For 14 bar model
            offset2 = offset2 + 7 * ((temp_calc+1500)*(temp_calc+1500));
            sensitivity2 = sensitivity2 + 4 * ((temp_calc+1500)*(temp_calc+1500));
        }
    } 
    else { // if TEMP is > 2000 (20.0C)
		// For 14 bar model
		dt2 = 7 * ((uint64_t)dt * dt) / 137438953472ULL; // = pow(2,37)
		//dt2 = (int32_t)dt2; // recast as signed 32bit integer
		offset2 = 1 * ((temp_calc-2000) * (temp_calc-2000)) / 16;
		sensitivity2 = 0;
    }
    
    // Calculate initial offset and sensitivity
    // Notice lots of casts to int64_t to ensure that the 
    // multiplication operations don't overflow the original 16 bit and 32 bit
    // integers
	offset = ((int64_t)prom_coefficient[2] << 16) + (((prom_coefficient[4] * (int64_t)dt)) >> 7);
	sensitivity = ((int64_t)prom_coefficient[1] << 15) + (((prom_coefficient[3] * (int64_t)dt)) >> 8);
    temp_calc = temp_calc - dt2; // both should be int32_t
    offset = offset - offset2; // both should be int64_t
    sensitivity = sensitivity - sensitivity2; // both should be int64_t
	pressure_calc = (((pressure_raw * sensitivity) / 2097152) - offset) / 32768;
    
    barometer->mbar = (float)(pressure_calc / 10.0f);
    barometer->temp_c = (float)(temp_calc / 100.0f);
    barometer->altitude_m = (float)(44307.7 * (1.0 - (pow(barometer->mbar / 1013.25, 0.190284))));
    barometer->altitude_cm = barometer->altitude_m * 10;
    
    barometer->altitude_ground_m = barometer->altitude_m - 10 * barometer->offset;
    barometer->altitude_ground_cm = barometer->altitude_cm - barometer->offset;
}

static void barometer_read_prom()
{
    uint8_t in_buffer[2];
    uint8_t i;
    for(i = 0; i < 8; i++){
        Sensors_I2C1_ReadRegister(MS5803_ADDRESS_HIGH, MS5803_CMD_PROM + (i << 1), 2, in_buffer);
        //I2C_TransmitByte(MS5803_ADDRESS_HIGH, MS5803_CMD_PROM + (i << 1));
        //I2C_Receive(MS5803_ADDRESS_HIGH, in_buffer, 2);
        prom_coefficient[i] = (uint16_t) (in_buffer[1] | ((uint16_t)in_buffer[0] << 8));
    }
}

static uint8_t barometer_validate_crc()
{
    int cnt;
    unsigned int n_rem;
    unsigned int crc_read;
    unsigned char  n_bit;
    
    n_rem = 0x00;
    crc_read = prom_coefficient[7];
    prom_coefficient[7] = ( 0xFF00 & ( prom_coefficient[7] ) );
    
    for (cnt = 0; cnt < 16; cnt++){ // choose LSB or MSB
        if ( cnt%2 == 1 ){
           n_rem ^= (unsigned short) ( ( prom_coefficient[cnt>>1] ) & 0x00FF );
        }
        else {
            n_rem ^= (unsigned short) ( prom_coefficient[cnt>>1] >> 8 );
        }
        for ( n_bit = 8; n_bit > 0; n_bit-- ){
            if ( n_rem & ( 0x8000 ) ){
                n_rem = ( n_rem << 1 ) ^ 0x3000;
            }
            else {
                n_rem = ( n_rem << 1 );
            }
        }
    }
    
    n_rem = ( 0x000F & ( n_rem >> 12 ) );// // final 4-bit reminder is CRC code
    prom_coefficient[7] = crc_read; // restore the crc_read to its original place
    
    // return (crc_read == (n_rem ^ 0x00));
    if(crc_read == (n_rem ^ 0x00)){
        return 1;
    }
    else{
        return 0;
    }
}

/*void barometer_task(void *pvParameters)
{
    UBaseType_t stack_size;
    stack_size = uxTaskGetStackHighWaterMark(NULL);
    barometer_init();
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t delay_ms = 65; // 1000ms / 5 = 200Hz

    while(1)
    {
        vTaskDelayUntil(&last_wake_time, delay_ms / portTICK_PERIOD_MS); // good shit!
        
        barometer_read();
   
        //printf2(" barometer:");
        //printf2(" mbar: %.4f", barometer.mbar);
        //printf2(" temp: %.4f", barometer.temp_c);
        printf2(" altitude: %.4f", barometer.altitude_m);
        printf2("\n\r");
        stack_size = uxTaskGetStackHighWaterMark(NULL);
        barometer.stack_size = stack_size;
    }
}*/
