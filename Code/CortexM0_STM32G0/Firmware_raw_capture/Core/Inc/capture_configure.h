#include "stm32g0xx.h"

#ifndef _CAPTURE_CONFIGURE_H
#define _CAPTURE_CONFIGURE_H

#define POINTS_TO_SAMPLE        250

//Tho channels
#define ADC_CAPURE_BUF_LENGTH   (POINTS_TO_SAMPLE * 2)

#define TIMER1_FREQ             SystemCoreClock
#define TIMER1_DIV              8


#define ADC_TRIGGER_FREQ_HZ     50000


#define TIMER1_PERIOD           (uint16_t)(TIMER1_FREQ / TIMER1_DIV / ADC_TRIGGER_FREQ_HZ)

void prepare_capture(void);
void start_adc_capture(void);
void sort_captured_data(void);


#endif
