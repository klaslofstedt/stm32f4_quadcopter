/***************************************************
  This is a library for the Adafruit VL53L0X Sensor Breakout

  Designed specifically to work with the VL53L0X sensor from Adafruit
  ----> https://www.adafruit.com/products/3317

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef VL53L0X_H
#define VL53L0X_H

#include "i2c_1.h"
#include "vl53l0x_api.h"

typedef struct{
    uint8_t vl53l0x_addr;
    /*VL53L0X_Error Status;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Version_t Version;
    VL53L0X_DeviceInfo_t DeviceInfo;*/
    float height_cm; // m?
} vl53l0x_data_t;   

#define VL53L0X_I2C_ADDR  0x29

void VL53L0X_init();
VL53L0X_Error rangingTest(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
VL53L0X_Error getSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
void VL53L0X_printRangeStatus(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);



#endif
