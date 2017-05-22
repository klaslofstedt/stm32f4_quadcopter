#ifndef UART_H
#define UART_H

void uart_printf(const char *format, ...);
void USART_puts(const char *s);
void USART_putc(char c);
void uart_init(void);

#endif


