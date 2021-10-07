#ifndef _UART_HANDLER_H
#define _UART_HANDLER_H
#include "stm32f0xx.h"


void init_uart1(void);
void uart_dma_start_tx(uint8_t* data_ptr, uint16_t length);


#endif











