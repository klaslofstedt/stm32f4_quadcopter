/*******************************************************************************
File    : i2c.c
Purpose : I2c 3 to communicate with the sensors
Author  : 
********************************** Includes ***********************************/
#include <stdio.h>
#include "stm32f4xx.h"
#include "i2c_1.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h" 
#include "stm32f4xx_i2c.h" 
#include "stm32f4xx_rcc.h"
//#include "board-st_discovery.h"
//freertos
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "freertos_time.h"

/********************************* Defines ************************************/

#define I2Cx_FLAG_TIMEOUT             ((uint32_t) 900) //0x1100
#define I2Cx_LONG_TIMEOUT             ((uint32_t) (300 * I2Cx_FLAG_TIMEOUT)) //was300
 
#define SENSORS_I2C                     I2C1
#define I2C_SPEED                       400000
#define I2C_OWN_ADDRESS                 0x00  // change??
#define SENSORS_I2C_SCL_GPIO_PORT       GPIOB
#define SENSORS_I2C_SCL_GPIO_CLK        RCC_AHB1Periph_GPIOB
#define SENSORS_I2C_SCL_GPIO_PIN        GPIO_Pin_6
#define SENSORS_I2C_SCL_GPIO_PINSOURCE  GPIO_PinSource6
 
#define SENSORS_I2C_SDA_GPIO_PORT       GPIOB
#define SENSORS_I2C_SDA_GPIO_CLK        RCC_AHB1Periph_GPIOB
#define SENSORS_I2C_SDA_GPIO_PIN        GPIO_Pin_7
#define SENSORS_I2C_SDA_GPIO_PINSOURCE  GPIO_PinSource7

#define SENSORS_I2C_RCC_CLK             RCC_APB1Periph_I2C1
#define SENSORS_I2C_AF                  GPIO_AF_I2C1


#define WAIT_FOR_FLAG(flag, value, timeout, errorcode)  I2CTimeout = timeout;\
          while(I2C_GetFlagStatus(SENSORS_I2C, flag) != value) {\
            if((I2CTimeout--) == 0) return I2C1x_TIMEOUT_UserCallback(errorcode); \
          }\
  
#define CLEAR_ADDR_BIT      I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR1);\
                            I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR2);\
                               
/********************************* Globals ************************************/
SemaphoreHandle_t g_i2c1_mutex;
/********************************* Prototypes *********************************/
unsigned long ST_Sensors_I2C1_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
unsigned long ST_Sensors_I2C1_WriteReg(unsigned char Address, unsigned char RegisterAddr, const unsigned char RegisterValue);
unsigned long ST_Sensors_I2C1_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
unsigned long ST_Sensors_I2C1_Write(unsigned char Address, const unsigned char RegisterValue);
unsigned long ST_Sensors_I2C1_Read(unsigned char Address, unsigned short RegisterLen, unsigned char *RegisterValue);
/*******************************  Function ************************************/


void i2c1_init(void)
{
  g_i2c1_mutex = xSemaphoreCreateMutex();  
    
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

    /* Enable I2Cx clock */
  RCC_APB1PeriphClockCmd(SENSORS_I2C_RCC_CLK, ENABLE);

  /* Enable I2C GPIO clock */
  RCC_AHB1PeriphClockCmd(SENSORS_I2C_SCL_GPIO_CLK | SENSORS_I2C_SDA_GPIO_CLK, ENABLE);

  /* Configure I2Cx pin: SCL ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin =  SENSORS_I2C_SCL_GPIO_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; //GPIO_PuPd_NOPULL;

  /* Connect pins to Periph */
  GPIO_PinAFConfig(SENSORS_I2C_SCL_GPIO_PORT, SENSORS_I2C_SCL_GPIO_PINSOURCE, SENSORS_I2C_AF);  
  GPIO_Init(SENSORS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Configure I2Cx pin: SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SDA_GPIO_PIN; 

  /* Connect pins to Periph */
  GPIO_PinAFConfig(SENSORS_I2C_SDA_GPIO_PORT, SENSORS_I2C_SDA_GPIO_PINSOURCE, SENSORS_I2C_AF);  
  GPIO_Init(SENSORS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);  
  
  I2C_DeInit(SENSORS_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    
  /* Enable the I2C peripheral */
  I2C_Cmd(SENSORS_I2C, ENABLE);  
    
  /* Initialize the I2C peripheral */
  I2C_Init(SENSORS_I2C, &I2C_InitStructure);
  
  return;
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
static uint32_t I2C1x_TIMEOUT_UserCallback(char value)
{
  
  /* The following code allows I2C error recovery and return to normal communication
     if the error source doesn’t still exist (ie. hardware issue..) */
  I2C_InitTypeDef I2C_InitStructure;
  
  I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2C, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2C, DISABLE);
  
  I2C_DeInit(SENSORS_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    
  /* Enable the I2C peripheral */
  I2C_Cmd(SENSORS_I2C, ENABLE);  
    
  /* Initialize the I2C peripheral */
  I2C_Init(SENSORS_I2C, &I2C_InitStructure);

  return 1;
}

int Sensors_I2C1_Write(unsigned char slave_addr, const unsigned char data_ptr)
{
  //xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();
                              
tryWriteAgain:  
  ret = 0;
  ret = ST_Sensors_I2C1_Write( slave_addr, data_ptr); 

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;
    
    delay_ms(retry_in_mlsec);
    goto tryWriteAgain;
  }
  //xSemaphoreGive(g_i2c1_mutex);
  return ret;  
}
int Sensors_I2C1_WriteReg(unsigned char slave_addr,
                                        unsigned char reg_addr, 
                                        const unsigned char data_ptr)
{
  //xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();
                              
tryWriteAgain:  
  ret = 0;
  ret = ST_Sensors_I2C1_WriteReg( slave_addr, reg_addr, data_ptr); 

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;
    
    delay_ms(retry_in_mlsec);
    goto tryWriteAgain;
  }
  //xSemaphoreGive(g_i2c1_mutex);
  return ret;  
}

int Sensors_I2C1_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        const unsigned char *data_ptr)
{
  //xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();
                              
tryWriteAgain:  
  ret = 0;
  ret = ST_Sensors_I2C1_WriteRegister( slave_addr, reg_addr, len, data_ptr); 

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;
    
    delay_ms(retry_in_mlsec);
    goto tryWriteAgain;
  }
  //xSemaphoreGive(g_i2c1_mutex);
  return ret;  
}


int Sensors_I2C1_Read(unsigned char slave_addr,
                                       unsigned short len, 
                                       unsigned char *data_ptr)
{
  //xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();
  
tryReadAgain:  
  ret = 0;
  ret = ST_Sensors_I2C1_Read(slave_addr, len, data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;
    
    delay_ms(retry_in_mlsec);
    goto tryReadAgain;
  } 
  //xSemaphoreGive(g_i2c1_mutex);
  return ret;
}


int Sensors_I2C1_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len, 
                                       unsigned char *data_ptr)
{
  //xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();
  
tryReadAgain:  
  ret = 0;
  ret = ST_Sensors_I2C1_ReadRegister( slave_addr, reg_addr, len, data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;
    
    delay_ms(retry_in_mlsec);
    goto tryReadAgain;
  } 
  //xSemaphoreGive(g_i2c1_mutex);
  return ret;
}


/**
  * @brief  Writes a Byte to a given register to the sensors through the 
            control interface (I2C)
  * @param  RegisterAddr: The address (location) of the register to be written.
  * @param  RegisterValue: the Byte value to be written into destination register.
  * @retval 0 if correct communication, else wrong communication
  */
unsigned long ST_Sensors_I2C1_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
  uint32_t  result = 0;
  uint32_t  i = 0; // i = RegisterLen;
  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
  
//  RegisterValue = RegisterValue + (RegisterLen - 1);

  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);
  
  /* Start the config sequence */
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
  
  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);

  /* Transmit the first address for write operation */
  I2C_SendData(SENSORS_I2C, RegisterAddr);

//  while (i--)
//  {
//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
//  
//    /* Prepare the register value to be sent */
//    I2C_SendData(SENSORS_I2C, *RegisterValue--);
//  }
  
  for(i=0; i<(RegisterLen); i++)
  {
    /* Wait for address bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
  
    /* Prepare the register value to be sent */
    I2C_SendData(SENSORS_I2C, RegisterValue[i]);
  }  
   
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);
  
  /* End the configuration sequence */
  I2C_GenerateSTOP(SENSORS_I2C, ENABLE);  
  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;  
}

unsigned long ST_Sensors_I2C1_WriteReg(unsigned char Address, unsigned char RegisterAddr, const unsigned char RegisterValue)
{
  uint32_t  result = 0;

  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
  
//  RegisterValue = RegisterValue + (RegisterLen - 1);

  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);
  
  /* Start the config sequence */
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
  
  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);

  /* Transmit the first address for write operation */
  I2C_SendData(SENSORS_I2C, RegisterAddr);

//  while (i--)
//  {
//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
//  
//    /* Prepare the register value to be sent */
//    I2C_SendData(SENSORS_I2C, *RegisterValue--);
//  }

    /* Wait for address bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
  
    /* Prepare the register value to be sent */
    I2C_SendData(SENSORS_I2C, RegisterValue);
   
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);
  
  /* End the configuration sequence */
  I2C_GenerateSTOP(SENSORS_I2C, ENABLE);  
  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;  
}

unsigned long ST_Sensors_I2C1_Write(unsigned char Address, const unsigned char RegisterValue)
{
  uint32_t  result = 0;

  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
  
//  RegisterValue = RegisterValue + (RegisterLen - 1);

  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);
  
  /* Start the config sequence */
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
  
  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);

  /* Transmit the first address for write operation */
  //I2C_SendData(SENSORS_I2C, RegisterAddr);

//  while (i--)
//  {
//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
//  
//    /* Prepare the register value to be sent */
//    I2C_SendData(SENSORS_I2C, *RegisterValue--);
//  }

    /* Wait for address bit to be set */
    //WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
  
    /* Prepare the register value to be sent */
    I2C_SendData(SENSORS_I2C, RegisterValue);
   
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);
  
  /* End the configuration sequence */
  I2C_GenerateSTOP(SENSORS_I2C, ENABLE);  
  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;  
}


unsigned long ST_Sensors_I2C1_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
  uint32_t i=0, result = 0;
  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
   
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 7);
  
  /* Start the config sequence */
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 8);
  
  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 9);

  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT;
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 10);
  
  /* Transmit the register address to be read */
  I2C_SendData(SENSORS_I2C, RegisterAddr);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 11);  

  /*!< Send START condition a second time */  
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);
  
  /*!< Send address for read */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Receiver);  
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);
  
  if (RegisterLen == 1) 
  {
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
    
    /* Wait for the RXNE bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);
    
    RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
  } 
  else if( RegisterLen == 2) 
  {
     /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    
   /* Set POS bit */ 
   SENSORS_I2C->CR1 |= I2C_CR1_POS;
   
   /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
   CLEAR_ADDR_BIT; 
   
   /* Wait for the buffer full bit to be set */
   WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);
   
   /*!< Send STOP Condition */
   I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

   /* Read 2 bytes */
   RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
   RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
  } 
  else if( RegisterLen == 3)
  {
    CLEAR_ADDR_BIT;
    
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    /* Read 1 bytes */
    RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);        
    /* Read 1 bytes */
    RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
    /* Read 1 bytes */
    RegisterValue[2] = I2C_ReceiveData(SENSORS_I2C);  
  }  
  else /* more than 2 bytes */
  { 
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    for(i=0; i<(RegisterLen); i++)
    {
      if(i==(RegisterLen-3))
      {
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
        
        /*!< Disable Acknowledgment */
        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
        
        /*!< Send STOP Condition */
        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);        
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
        
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);  
        goto endReadLoop;
      }
            
      /* Wait for the RXNE bit to be set */
      WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
      RegisterValue[i] = I2C_ReceiveData(SENSORS_I2C); 
    }   
  } 
  
endReadLoop:  
  /* Clear BTF flag */
  I2C_ClearFlag(SENSORS_I2C, I2C_FLAG_BTF);
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);  
  /*!< Re-Enable Acknowledgment to be ready for another reception */
  I2C_AcknowledgeConfig(SENSORS_I2C, ENABLE);
  //Disable POS -- TODO
  SENSORS_I2C->CR1 &= ~I2C_CR1_POS;  
     
  /* Return the byte read from sensor */
  return result;
}

unsigned long ST_Sensors_I2C1_Read(unsigned char Address, unsigned short RegisterLen, unsigned char *RegisterValue)
{
  uint32_t i=0, result = 0;
  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
     
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 11);  

  /*!< Send START condition a second time */  
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);
  
  /*!< Send address for read */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Receiver);  
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);
  
  if (RegisterLen == 1) 
  {
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
    
    /* Wait for the RXNE bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);
    
    RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
  } 
  else if( RegisterLen == 2) 
  {
     /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    
   /* Set POS bit */ 
   SENSORS_I2C->CR1 |= I2C_CR1_POS;
   
   /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
   CLEAR_ADDR_BIT; 
   
   /* Wait for the buffer full bit to be set */
   WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);
   
   /*!< Send STOP Condition */
   I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

   /* Read 2 bytes */
   RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
   RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
  } 
  else if( RegisterLen == 3)
  {
    CLEAR_ADDR_BIT;
    
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    /* Read 1 bytes */
    RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);        
    /* Read 1 bytes */
    RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
    /* Read 1 bytes */
    RegisterValue[2] = I2C_ReceiveData(SENSORS_I2C);  
  }  
  else /* more than 2 bytes */
  { 
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    for(i=0; i<(RegisterLen); i++)
    {
      if(i==(RegisterLen-3))
      {
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
        
        /*!< Disable Acknowledgment */
        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
        
        /*!< Send STOP Condition */
        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);        
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
        
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);  
        goto endReadLoop;
      }
            
      /* Wait for the RXNE bit to be set */
      WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
      RegisterValue[i] = I2C_ReceiveData(SENSORS_I2C); 
    }   
  } 
  
endReadLoop:  
  /* Clear BTF flag */
  I2C_ClearFlag(SENSORS_I2C, I2C_FLAG_BTF);
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);  
  /*!< Re-Enable Acknowledgment to be ready for another reception */
  I2C_AcknowledgeConfig(SENSORS_I2C, ENABLE);
  //Disable POS -- TODO
  SENSORS_I2C->CR1 &= ~I2C_CR1_POS;  
     
  /* Return the byte read from sensor */
  return result;
}


static unsigned short RETRY_IN_MLSEC  = 5;

void Set_I2C_Retry1(unsigned short ml_sec)
{
  RETRY_IN_MLSEC = ml_sec;
}

unsigned short Get_I2C1_Retry()
{
  return RETRY_IN_MLSEC;
}



/*#include "i2c_1.h"
#include "uart.h"
#include "freertos_time.h"

static void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
static uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
static uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
static void I2C_stop(I2C_TypeDef* I2Cx);
static void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);


void I2C_TransmitArray(uint8_t address, uint8_t *out_buffer, uint8_t length)
{
    I2C_start(I2C1, address<<1, I2C_Direction_Transmitter);
    int i;
    for(i = 0; i < length; i++){     
        I2C_write(I2C1, out_buffer[i]); // write one byte to the slave
    }
    I2C_stop(I2C1); // stop the transmission
}
void I2C_TransmitByte(uint8_t address, uint8_t out_buffer)
{
    //printf2("i2c0");
    I2C_start(I2C1, address<<1, I2C_Direction_Transmitter);
    //delay_ms(1); // ta bort va?
    //printf2("i2c1");
    I2C_write(I2C1, out_buffer); // write another byte to the slave
    //printf2("i2c2");
    I2C_stop(I2C1); // stop the transmission
    //printf2("i2c3");
}

void I2C_TransmitReg(uint8_t address, uint8_t reg, uint8_t out_buffer)
{
    
    //printf2("u");
    I2C_start(I2C1, address<<1, I2C_Direction_Transmitter);
    delay_ms(1);
    //printf2("v");
    I2C_write(I2C1, reg); // write one byte to the slave
    //printf2("w");
    delay_ms(1);
    I2C_write(I2C1, out_buffer); // write another byte to the slave
    delay_ms(1);
    //printf2("x");
    I2C_stop(I2C1); // stop the transmission
    //printf2("y");
}

void I2C_TransmitMultiReg(uint8_t address, uint8_t reg, uint8_t *out_buffer, uint32_t length)
{
    printf2("z");
    I2C_start(I2C1, address<<1, I2C_Direction_Transmitter);
    //printf2("v");
    I2C_write(I2C1, reg); // write one byte to the slave
    int i;
    for(i = 0; i < length; i++){     
        I2C_write(I2C1, out_buffer[i]); // write one byte to the slave
        delay_ms(1);
        //printf2("q\n\r");
    }
    I2C_stop(I2C1); // stop the transmission
}

void I2C_Receive(uint8_t address, uint8_t *in_buffer, uint8_t length)
{
    // start a transmission in Master receiver mode
    I2C_start(I2C1, address<<1, I2C_Direction_Receiver);
    int i;
    for(i = 0; i < length-1; i++){
        in_buffer[i] = I2C_read_ack(I2C1);
    }
    in_buffer[length-1] = I2C_read_nack(I2C1);
    //I2C_stop(I2C1);
}


void i2c1_init(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
	
	// configure I2C1 
	I2C_InitStruct.I2C_ClockSpeed = 400000; 		// 100kHz // 400?
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

static void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
    //delay_ms(1);
	// wait until I2C1 is not busy any more
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    //WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1); //:klas
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    //WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2); // :klas
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}


static void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
    //delay_ms(1);
	// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2Cx, data);
}


static uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
    //delay_ms(1);
	// enable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

static uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
    //delay_ms(1);
	// disable acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}


static void I2C_stop(I2C_TypeDef* I2Cx)
{
	//delay_ms(1);
	// Send I2C1 STOP Condition after last byte has been transmitted
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}*/