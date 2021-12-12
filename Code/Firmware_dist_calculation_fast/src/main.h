#ifndef _MAIN_H
#define _MAIN_H

extern volatile uint32_t uwTick;

//512A board revision is selected if MODULE_701A not defined
#define MODULE_701A
//There are exists confirmations that 703A module works with MODULE_701A firmware


//#define FAST_CAPTURE

//10 KHz mode
//#define DUAL_FREQUENCY

//Use key3 pin as output
//#define DEBUG_PIN

#define ENHANCED_APD_CALIBADION

/* Exported macro ------------------------------------------------------------*/
#define START_TIMER(x, duration)  (x = (uwTick + duration))
#define TIMER_ELAPSED(x)  ((uwTick > x) ? 1 : 0)

void Delay_ms(uint32_t ms);

void process_rx_data(uint8_t data);
void auto_switch_apd_voltage(uint16_t current_amplitude);

#endif
