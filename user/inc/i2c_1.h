#ifndef I2C_1_H
#define I2C_1_H

#include <stdint.h>
#include "stm32f4xx_conf.h"
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>


void I2C_TransmitReg(uint8_t address, uint8_t reg, uint8_t out_buffer);
void I2C_TransmitByte(uint8_t address, uint8_t out_buffer);
//void I2C_TransmitArray(uint8_t address, uint8_t *out_buffer, uint8_t lenght);
//void I2C_Receive(uint8_t address, uint8_t reg, uint8_t *in_buffer, uint8_t lenght);
void I2C_Receive(uint8_t address, uint8_t *in_buffer, uint8_t lenght);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);
void i2c1_init(void);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);

#endif
