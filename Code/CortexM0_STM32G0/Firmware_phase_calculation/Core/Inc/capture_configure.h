#include "stm32g0xx.h"
#include "analyse.h"

#ifndef _CAPTURE_CONFIGURE_H
#define _CAPTURE_CONFIGURE_H

#define POINTS_TO_SAMPLE        250

//Tho channels
#define ADC_CAPURE_BUF_LENGTH   (POINTS_TO_SAMPLE * 2)

#define TIMER1_FREQ             SystemCoreClock
#define TIMER1_DIV              8

//ADC trigger sample rate (2 channes per one trigger event)
#define ADC_TRIGGER_FREQ_HZ     50000

#define TIMER1_PERIOD           (uint16_t)(TIMER1_FREQ / TIMER1_DIV / ADC_TRIGGER_FREQ_HZ)

#define REPEAT_NUMBER           1 //number of averaging points
#define SIGNAL_FREQ_HZ          5000 //APD mixed signal and reference signal frequency, Hz

//Goertzel coefficent - MUST be integer, or quality fall
#define K_COEF    ((float)SIGNAL_FREQ_HZ * (float)POINTS_TO_SAMPLE / (float)ADC_TRIGGER_FREQ_HZ)
  
void prepare_capture(void);
void start_adc_capture(void);
void sort_captured_data(void);
AnalyseResultType do_capture(void);

#endif
