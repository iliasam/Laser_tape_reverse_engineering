#ifndef _MAIN_H
#define _MAIN_H

#define MODULE_701A

//#define FAST_CAPTURE

void Delay_ms(uint32_t ms);

void process_rx_data(uint8_t data);
void do_triple_phase_measurement(void);

#endif
