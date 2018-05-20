#include "stm32f10x.h"

#ifndef _ANALYSE_H
#define _ANALYSE_H

#define ABS(x)         (x < 0) ? (-x) : x

//phase value is in bounds [0 ... MAX_ANGLE*PHASE_MULT]
#define MAX_ANGLE       360
#define HMAX_ANGLE      (MAX_ANGLE / 2) //half of maximum angle
#define PHASE_MULT      10              //phase multiply coefficient

#define ZERO_ANGLE1     (MAX_ANGLE / 24)//close to zero angle value
#define ZERO_ANGLE2     (MAX_ANGLE - (MAX_ANGLE / 24))//close to zero angle value

#define CORR_360_DEG    1.0 // = MAX_ANGLE / 360.0

typedef struct
{
   uint16_t Amplitude;
   int16_t Phase;//deg*10
} AnalyseResultType;

void init_goertzel(void);
AnalyseResultType goertzel_analyse(uint16_t* data);
//AnalyseResultType do_result_conversion(GoertzelResultType point);
AnalyseResultType do_result_conversion(float real, float imag);
int16_t calculate_avr_phase(int16_t* data, uint16_t length);

int16_t calculate_median_phase(int16_t* data, uint16_t length);
int16_t calculate_correction(uint16_t raw_temperature, uint16_t amplitude, uint8_t apd_voltage);
uint8_t phase_close_to_zero(int16_t phase);

#endif
