#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

/* Demo program include files. */
#include "uart.h"
#include "freertos_time.h"
#include "imu2.h"
#include "i2c.h"
#include "ekf2.h"
#include "gpio.h"
#include "board.h"

// dmp

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_it.h"



#define MPU6050_ADDRESS             0x68 // 0x68 or 0x69
#define BAROMETRIC_ADDRESS          0x77
#define MAGNETOMETER_ADDRESS        0x1E

#define USER_CTRL                   0x6A
#define MASTER_AND_FIFO_DISABLED    0x00

#define BYPASS_MODE                 0x37
#define BYPASS_ON                   0x02
#define BYPASS_OFF                  0x00

#define PWR_MGMT_1          0x6B
/* these names makes no sense
should be something with clock souce PLL */
#define PLL_X_GYRO          0x01
#define PLL_Y_GYRO          0x02
#define PLL_Z_GYRO          0x03

#define SMPRT_DIV           0x19        // sample rate = 1(kHz)/(1+SMPLTR_DIV)
#define RATE500Hz           0x01
#define RATE333Hz           0x02
#define RATE250Hz           0x03
#define RATE200Hz           0x04
#define RATE167Hz           0x05
#define RATE143Hz           0x06
#define RATE125Hz           0x07
#define RATE125Hz           0x08
#define RATE111Hz           0x09
#define RATE100Hz           0x0A

#define CONFIG              0x1A
#define BW260HZ             0x00
#define BW185HZ             0x01
#define BW95HZ              0x02
#define BW44HZ              0x03
#define BW20HZ              0x04

#define GYRO_CONFIG         0x1B
#define GYRO_RANGE_250      0x00
#define GYRO_RANGE_500      0x08
#define GYRO_RANGE_1000     0x10
#define GYRO_RANGE_2000     0x18

#define ACCEL_CONFIG        0x1C
#define ACCEL_RAGE_2G       0x00      //16384 LSB/g
#define ACCEL_RAGE_4G       0x08
#define ACCEL_RAGE_8G       0x10
#define ACCEL_RAGE_10G      0x18

#define ACCEL_XOUT     0x3B                // Accelerometer output data registers
#define GYRO_XOUT      0x43                // Gyroscopes output data registers

#define MPU6050_WHO_AM_I        0x75

#define GYRO_X_SCALE_RAD 0.00026646248//0.01526717557 (in degrees)
#define GYRO_Y_SCALE_RAD 0.00026646248//0.01526717557
#define GYRO_Z_SCALE_RAD 0.00026646248//0.01526717557
#define GYRO_X_SCALE_DEG 0.01526717557 //(in degrees)
#define GYRO_Y_SCALE_DEG 0.01526717557
#define GYRO_Z_SCALE_DEG 0.01526717557

#define GYRO_AVERAGE_OFFSET_X 0
#define GYRO_AVERAGE_OFFSET_Y 0
#define GYRO_AVERAGE_OFFSET_Z 0

#define ACCEL_X_SCALE 0.00006103515
#define ACCEL_Y_SCALE 0.00006103515
#define ACCEL_Z_SCALE 0.00006103515
#define ACCEL_X_OFFSET 0
#define ACCEL_Y_OFFSET 0
#define ACCEL_Z_OFFSET 0

#define M_PI   3.14159265358979323846264338327950288

#define LOOP_TIME_MS 3

xSemaphoreHandle imu_attitude_sem = NULL;
xSemaphoreHandle imu_altitude_sem = NULL;

xQueueHandle imu_attitude_queue = 0;
xQueueHandle imu_altitude_queue = 0;


imu_data_t imu = {
    .angle_x = 0,
    .angle_y = 0,
    .angle_z = 0,
    .rate_x = 0,
    .rate_y = 0,
    .rate_z = 0,
    .gyro_y = 0,
    .gyro_z = 0,
    .acc_x = 0,
    .acc_y = 0,
    .acc_z = 0,
    .dt = 0,
    .stack_size = 0
};

ekf2_data_t ekf2_roll = {
    /* We will set the variables like so, these can also be tuned by the user */
    .Q_angle = 0.0000009386,
    .Q_bias = 0.0f,
    .R_measure = 3.89f,

    .angle = 0.0f, // Reset the angle
    .rate = 0.0f,
    .bias = 0.0f, // Reset bias

    .P[0][0] = 1.0f, // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    .P[0][1] = 0.0f,
    .P[1][0] = 0.0f,
    .P[1][1] = 1.0f
};

ekf2_data_t ekf2_pitch = {
    /* We will set the variables like so, these can also be tuned by the user */
    .Q_angle = 0.0000009386f,
    .Q_bias = 0.0f,
    .R_measure = 3.89f,

    .angle = 0.0f, // Reset the angle
    .rate = 0.0f,
    .bias = 0.0f, // Reset bias
    // Change these to "don't know starting angle"
    .P[0][0] = 1.0f, // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    .P[0][1] = 0.0f,
    .P[1][0] = 0.0f,
    .P[1][1] = 1.0f
};

static void imu2_init(void);
static void MPU6050_WriteRegiser(uint8_t RegAdress, uint8_t RegData);
static void imu2_read_acc(void);
static void imu2_read_gyr(void);

static void imu2_init(void)
{
    delay_ms(500);
    MPU6050_WriteRegiser(PWR_MGMT_1, PLL_Z_GYRO);
    MPU6050_WriteRegiser(USER_CTRL, MASTER_AND_FIFO_DISABLED);
    MPU6050_WriteRegiser(BYPASS_MODE, BYPASS_ON);
    MPU6050_WriteRegiser(SMPRT_DIV, RATE333Hz);
    MPU6050_WriteRegiser(CONFIG, BW20HZ);
    MPU6050_WriteRegiser(GYRO_CONFIG, GYRO_RANGE_500);
    MPU6050_WriteRegiser(ACCEL_CONFIG, ACCEL_RAGE_2G);
    delay_ms(500);
}

static void MPU6050_WriteRegiser(uint8_t RegAdress, uint8_t RegData)
{
    Sensors_I2C_WriteReg(MPU6050_ADDRESS, RegAdress, RegData);
    // start a transmission in Master transmitter mode
    /*I2C_start(I2C1, MPU6050_ADDRESS<<1, I2C_Direction_Transmitter);
    
	I2C_write(I2C1, RegAdress); // write one byte to the slave
	I2C_write(I2C1, RegData); // write another byte to the slave
	I2C_stop(I2C1); // stop the transmission
    Delay(200000);*/
}

static void imu2_read_acc(void)
{
	uint8_t buffer[6];
    int16_t racc[3];
    
    Sensors_I2C_ReadRegister(MPU6050_ADDRESS, ACCEL_XOUT, 6, buffer);
    
    
    // Now multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    racc[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
    racc[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
    racc[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)
    
    // Compensate accelerometer error
    // put this in a seperate inline void funcion
    imu.acc_x = -(racc[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    imu.acc_y = -(racc[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    imu.acc_z = -(racc[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
    
}

static void imu2_read_gyr(void)
{
	uint8_t buffer[6];
    int16_t rgyr[3];
    
    Sensors_I2C_ReadRegister(MPU6050_ADDRESS, GYRO_XOUT, 6, buffer);
    
    // Now multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    rgyr[0] = (((int16_t) buffer[0]) << 8) | buffer[1];  // X axis (internal sensor y axis)
    rgyr[1] = (((int16_t) buffer[2]) << 8) | buffer[3];  // Y axis (internal sensor x axis)
    rgyr[2] = (((int16_t) buffer[4]) << 8) | buffer[5];  // Z axis (internal sensor z axis)
    
    // Compensate accelerometer error
    // put this in a seperate inline void funcion
    /*gyr->Roll = GYRO_X_SCALE*rgyr[0];
    gyr->Roll -= GYRO_AVERAGE_OFFSET_X;
    gyr->Pitch = GYRO_Y_SCALE*rgyr[1];
    gyr->Pitch -= GYRO_AVERAGE_OFFSET_Y;
    gyr->Yaw = GYRO_Z_SCALE*rgyr[2];
    gyr->Yaw -= GYRO_AVERAGE_OFFSET_Z;*/
    
    imu.gyro_x = GYRO_X_SCALE_RAD*rgyr[0] - GYRO_AVERAGE_OFFSET_X;
    imu.gyro_y = GYRO_Y_SCALE_RAD*rgyr[1] - GYRO_AVERAGE_OFFSET_Y;
    imu.gyro_z = GYRO_Z_SCALE_RAD*rgyr[2] - GYRO_AVERAGE_OFFSET_Z;
}


void imu2_task(void *pvParameters)
{
    uart_printf("imu2 task\n\r");
    
    UBaseType_t stack_size;
    stack_size = uxTaskGetStackHighWaterMark(NULL);
    
    imu2_init();
    
    imu_attitude_queue = xQueueCreate(1, sizeof(imu_data_t));
    imu_altitude_queue = xQueueCreate(1, sizeof(imu_data_t));
   
    //vSemaphoreCreateBinary(imu_attitude_sem);
    //vSemaphoreCreateBinary(imu_altitude_sem);
    
   
    TickType_t wake_time = xTaskGetTickCount();
    TickType_t last_wake_time = 0;
    const TickType_t delay = LOOP_TIME_MS; // 25 ms = 40 hz
    while(1)
    { 
        vTaskDelayUntil(&wake_time, LOOP_TIME_MS); // 250Hz
        GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_IMU_TASK_PIN);
        
        // read acceleration and gyro
        imu2_read_acc();
        imu2_read_gyr();
        // calc angle and rate for roll
        ekf2_roll.newAngle = imu.acc_x;
        ekf2_roll.newRate = imu.gyro_y; // not x for reasons
        ekf2_roll.dt = 0.00333333333;
        ekf2_calc(&ekf2_roll);
        imu.angle_x = ekf2_roll.angle * (180 / M_PI);
        imu.rate_x = ekf2_roll.rate * (180 / M_PI);
        
        // calc angle and rate for pitch
        ekf2_pitch.newAngle = imu.acc_y;
        ekf2_pitch.newRate = imu.gyro_x;
        ekf2_pitch.dt = 0.00333333333;
        ekf2_calc(&ekf2_pitch);
        imu.angle_y = ekf2_pitch.angle * (180 / M_PI);
        imu.rate_y = ekf2_pitch.rate * (180 / M_PI);
        
        
        GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_IMU_TASK_PIN);
        xQueueOverwrite(imu_attitude_queue, &imu);
        
        stack_size = uxTaskGetStackHighWaterMark(NULL);
        imu.stack_size = stack_size;
        
    }
}