#ifndef _MAIN_H
#define _MAIN_H

//512A board revision if not selected
#define MODULE_701A

void Delay_ms(uint32_t ms);
void uart_send_data(uint8_t* data, uint16_t length);

#endif
