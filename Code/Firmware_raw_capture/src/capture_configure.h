#include "stm32f10x.h"

#ifndef _CAPTURE_CONFIGURE_H
#define _CAPTURE_CONFIGURE_H

#define POINTS_TO_SAMPLE 250

#define ADC_CAPURE_BUF_LENGTH (POINTS_TO_SAMPLE*2)

#define TIMER1_FREQ             24000000
#define TIMER1_DIV              6
#define ADC_TRIGGER_FREQ        50000
#define TIMER1_PERIOD           (uint16_t)(TIMER1_FREQ / TIMER1_DIV / ADC_TRIGGER_FREQ)

void prepare_capture(void);
void start_adc_capture(void);
void sort_captured_data(void);


#endif
