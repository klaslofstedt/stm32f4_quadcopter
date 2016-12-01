#include "USART.h"
#include <stm32f4xx_usart.h>
//#include "USB.h"
uint8_t USART1_RxByte = 0;

void USART1_IRQHandler(void)
{
	//check the type of interrupt to make sure we have received some data.
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		USART1_RxByte = USART1->DR; //Read the character that we have received
		USART1_SendByte(USART1_RxByte);
	}
}

void USART1_SendByte(volatile uint8_t s)
{
	while( !(USART1->SR & 0x00000040) );
	USART_SendData(USART1, s);
}

void USART1_SendString(volatile char *s)
{
	while(*s){
		// wait until data register is empty
		while( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, *s);
		*s++;
	}
}

void USART1_Init(void)
{
	GPIO_InitTypeDef 	GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	//Enable clock for USART1 peripheral
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStruct);

	//Enable RX interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
}
