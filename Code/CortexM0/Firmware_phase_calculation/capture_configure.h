#ifndef _CAPTURE_CONFIGURE_H
#define _CAPTURE_CONFIGURE_H

#include "stm32f0xx.h"
#include "analyse.h"

#define POINTS_TO_SAMPLE        250

#define ADC_CAPURE_BUF_LENGTH   (POINTS_TO_SAMPLE * 2)

#define TIMER1_FREQ             SystemCoreClock
#define TIMER1_DIV              6


#define ADC_TRIGGER_FREQ_HZ     50000

#define REPEAT_NUMBER           1 //number of averaging points
#define SIGNAL_FREQ             5000 //APD signal and reference signal frequency, Hz

//Goertzel coefficent - MUST be integer, or quality fall
#define K_COEF                  ((float)SIGNAL_FREQ * (float)POINTS_TO_SAMPLE / (float)ADC_TRIGGER_FREQ_HZ)

#define TIMER1_PERIOD           (uint16_t)(TIMER1_FREQ / TIMER1_DIV / ADC_TRIGGER_FREQ_HZ)

void prepare_capture(void);
void start_adc_capture(void);
void sort_captured_data(void);
void capture_init_adc_single_measure(void);
void capture_do_single_adc_measurements(void);

AnalyseResultType do_capture(void);


#endif
