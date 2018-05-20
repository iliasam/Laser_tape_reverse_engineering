#include "stm32f10x.h"
#include "main.h"
#include "analyse.h"


#ifndef _CAPTURE_CONFIGURE_H
#define _CAPTURE_CONFIGURE_H

#ifdef FAST_CAPTURE
  #define ADC_TRIGGER_FREQ      350000//sampling frequency, Hz
  #define ADC_SAMPLING_TIME     ADC_SampleTime_1Cycles5
  #define POINTS_TO_SAMPLE      140

  //for 5khz
  //#define ADC_TRIGGER_FREQ      250000//sampling frequency, Hz
  //#define POINTS_TO_SAMPLE      150
  
#else
  #define ADC_TRIGGER_FREQ      50000//sampling frequency, Hz
  #define ADC_SAMPLING_TIME     ADC_SampleTime_41Cycles5
  
  //250 -> ~57Hz distance measurements per second
  //120 -> ~100Hz
  #define POINTS_TO_SAMPLE      250  //number of sampled points in single capture - key parameter of this program
  
#endif

#ifdef DUAL_FREQUENCY
  #define SIGNAL_FREQ           10000 //APD signal and reference signal frequency, Hz
  #define SWITCH_DELAY          150 //time in uS to switch frequency
#else
  #define SIGNAL_FREQ           5000 //APD signal and reference signal frequency, Hz
  #define SWITCH_DELAY          300 //time in uS to switch frequency
#endif



#define ADC_CAPURE_BUF_LENGTH   (POINTS_TO_SAMPLE * 2)//signal points + reference points

#define TIMER1_FREQ             24000000 //Hz
#define TIMER1_DIV              1
#define TIMER1_PERIOD           (uint16_t)(TIMER1_FREQ / TIMER1_DIV / ADC_TRIGGER_FREQ)



//#define K_COEF  25
#define K_COEF                  (SIGNAL_FREQ*POINTS_TO_SAMPLE/ADC_TRIGGER_FREQ) //Goertzel coefficent - MUST be integer, or quality fall            

void switch_apd_voltage(uint8_t new_voltage);
void prepare_capture(void);
void start_adc_capture(uint16_t* data_ptr);


#endif
