#ifndef _MAIN_H
#define _MAIN_H

//512A board revision is selected if MODULE_701A not defined
#define MODULE_701A

//Testing
//#define FAST_CAPTURE

void Delay_ms(uint32_t ms);
void uart_send_data(uint8_t* data, uint16_t length);

#endif
