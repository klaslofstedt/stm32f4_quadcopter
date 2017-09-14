
#include <stdio.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h" 
#include "misc.h" 
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include <stdarg.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "semphr.h"

/********************************* Defines ************************************/


//////////////////  USART2
#define USARTx                           USART3
#define USARTx_CLK                       RCC_APB1Periph_USART3
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_8
#define USARTx_TX_GPIO_PORT              GPIOD
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOD
#define USARTx_TX_SOURCE                 GPIO_PinSource8
#define USARTx_TX_AF                     GPIO_AF_USART3

#define USARTx_RX_PIN                    GPIO_Pin_9
#define USARTx_RX_GPIO_PORT              GPIOD
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOD
#define USARTx_RX_SOURCE                 GPIO_PinSource9
#define USARTx_RX_AF                     GPIO_AF_USART3

#define USARTx_DMAx_CLK                  RCC_AHBPeriph_DMA1

/********************************* Globals ************************************/
SemaphoreHandle_t g_uart_mutex;
/********************************* Prototypes *********************************/
/*******************************  Function ************************************/



void uart_init(void)
{
  g_uart_mutex = xSemaphoreCreateMutex();
  
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN | USARTx_RX_PIN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  //GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  //GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  //GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  //GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
 
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

/******************************************************************************/

void USART_putc(char c)
{
  while(!(USARTx->SR & 0x00000040)); //?????????????
  USART_SendData(USART3,c);
}

void USART_puts(const char *s)
{
  int i;
  for(i=0;s[i]!=0;i++) USART_putc(s[i]);
}

void uart_printf(const char *format, ...) 
{
  xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
  
  va_list list;
  va_start(list, format);
  int len = vsnprintf(0, 0, format, list);
  char *s;
  s = (char *)malloc(len + 1);
  vsprintf(s, format, list);
  USART_puts(s);
  free(s);
  va_end(list);

  xSemaphoreGive(g_uart_mutex);
  return;
}

/******************************************************************************/

