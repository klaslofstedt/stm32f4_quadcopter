#ifndef __I2C_1_H__
#define __I2C_1_H__

//#define I2C_Config() I2cMaster_Init()

void i2c1_init(void);
void Set_I2C1_Retry(unsigned short ml_sec);
unsigned short Get_I2C_Retry();

int Sensors_I2C1_ReadRegister(unsigned char Address, unsigned char RegisterAddr, 
                                          unsigned short RegisterLen, unsigned char *RegisterValue);
int Sensors_I2C1_WriteRegister(unsigned char Address, unsigned char RegisterAddr, // multiple registers
                                           unsigned short RegisterLen, const unsigned char *RegisterValue);
int Sensors_I2C1_WriteReg(unsigned char Address, unsigned char RegisterAddr, // single register
                                           const unsigned char RegisterValue);
int Sensors_I2C1_Write(unsigned char slave_addr, const unsigned char data_ptr);
 
#endif // __I2C_H__
/*#ifndef I2C_1_H
#define I2C_1_H

#include <stdint.h>
#include "stm32f4xx_conf.h"
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>


void I2C_TransmitReg(uint8_t address, uint8_t reg, uint8_t out_buffer);
void I2C_TransmitMultiReg(uint8_t address, uint8_t reg, uint8_t *out_buffer, uint32_t length);
void I2C_TransmitByte(uint8_t address, uint8_t out_buffer);
void I2C_TransmitArray(uint8_t address, uint8_t *out_buffer, uint8_t lenght);
//void I2C_Receive(uint8_t address, uint8_t reg, uint8_t *in_buffer, uint8_t lenght);
void I2C_Receive(uint8_t address, uint8_t *in_buffer, uint8_t lenght);
void i2c1_init(void);


#endif*/
