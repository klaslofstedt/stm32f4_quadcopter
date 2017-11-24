#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"
#include "i2c_1.h"

//#define I2C_DEBUG

int VL53L0X_i2c_init(void) 
{
    // already initialized
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data) 
{
    Sensors_I2C1_WriteReg(deviceAddress, index, data);
    //I2C_TransmitReg(deviceAddress, index, data);
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) 
{
    //I2C_TransmitMultiReg(deviceAddress, index, pdata, count);
    Sensors_I2C1_WriteRegister(deviceAddress, index, count, pdata);
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data) 
{
    uint8_t buff[2];
    buff[1] = (uint8_t)(data & 0xFF);
    buff[0] = (uint8_t)(data >> 8);
    Sensors_I2C1_WriteRegister(deviceAddress, index, 2, buff);
    //I2C_TransmitMultiReg(deviceAddress, index, buff, 2);
    return VL53L0X_ERROR_NONE;
    //return VL53L0X_write_multi(deviceAddress, index, buff, 2);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data) 
{
    uint8_t buff[4];
    
    buff[3] = (uint8_t)(data & 0xFF);
    buff[2] = (uint8_t)(data >> 8);
    buff[1] = (uint8_t)(data >> 16);
    buff[0] = (uint8_t)(data >> 24);
    //I2C_TransmitMultiReg(deviceAddress, index, buff, 4);
    Sensors_I2C1_WriteRegister(deviceAddress, index, 4, buff);
    return VL53L0X_ERROR_NONE;
    //return VL53L0X_write_multi(deviceAddress, index, buff, 4);
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data) 
{
    //I2C_TransmitByte(deviceAddress, index);
    //I2C_Receive(deviceAddress, data, 1);
    Sensors_I2C1_ReadRegister(deviceAddress, index, 1, data);
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) 
{
    //I2C_TransmitByte(deviceAddress, index);
    //I2C_Receive(deviceAddress, pdata, count);
    Sensors_I2C1_ReadRegister(deviceAddress, index, count, pdata);
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data) 
{
    //I2C_TransmitByte(deviceAddress, index);
    uint8_t buff[2];
    //I2C_Receive(deviceAddress, buff, 2);
    Sensors_I2C1_ReadRegister(deviceAddress, index, 2, buff);
    
    uint16_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    *data = tmp;
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data) 
{
    //I2C_TransmitByte(deviceAddress, index);
    uint8_t buff[4];
    //I2C_Receive(deviceAddress, buff, 4);
    Sensors_I2C1_ReadRegister(deviceAddress, index, 4, buff);
    
    uint32_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    tmp <<= 8;
    tmp |= buff[2];
    tmp <<= 8;
    tmp |= buff[3];
    
    *data = tmp;
    
    return VL53L0X_ERROR_NONE;
}
