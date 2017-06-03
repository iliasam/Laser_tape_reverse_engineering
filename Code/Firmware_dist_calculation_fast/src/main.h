#ifndef _MAIN_H
#define _MAIN_H

void Delay_ms(uint32_t ms);
void uart_send_data(uint8_t* data, uint16_t length);

void process_rx_data(uint8_t data);
void auto_switch_apd_voltage(uint16_t current_amplitude);

#endif
