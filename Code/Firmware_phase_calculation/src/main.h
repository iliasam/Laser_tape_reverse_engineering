#ifndef _MAIN_H
#define _MAIN_H

void Delay_ms(uint32_t ms);

void process_rx_data(uint8_t data);
void do_triple_phase_measurement(void);
void auto_switch_apd_voltage(uint16_t current_amplitude);

#endif
