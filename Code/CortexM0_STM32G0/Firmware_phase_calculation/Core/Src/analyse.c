#include "stm32g0xx_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "math.h"
#include "analyse.h"
#include "stdlib.h"

#ifndef M_PI
#define M_PI            3.1415926535f
#endif

#define OMEGA ((2.0 * M_PI * K_COEF) / POINTS_TO_SAMPLE)

float sin_coef = 0.0f;
float cos_coef = 0.0f;
float g_coef = 0.0f;//goertzel coefficient


void init_goertzel(void)
{
  sin_coef = sin(OMEGA);
  cos_coef = cos(OMEGA);
  g_coef = 2.0f * cos_coef;
}

//data goes like data[used] + data[not used] + data[used] ...
AnalyseResultType goertzel_analyse(uint16_t* data)
{
  uint16_t i;
  float q0, q1, q2;
  float real, imag;
  float   scalingFactor = (float)POINTS_TO_SAMPLE / 2.0;
  AnalyseResultType result;
  
  q0=0;
  q1=0;//n-1
  q2=0;//n-2
  
  for(i=0; i < POINTS_TO_SAMPLE; i++)
  {
    q0 = g_coef * q1 - q2 + (float)data[i*2];
    q2 = q1;
    q1 = q0;
  }
  
  real = (q1 - q2 * cos_coef)/ scalingFactor;
  imag = (q2 * sin_coef)/ scalingFactor;
  result = do_result_conversion(real, imag);
  return result;
}

//convert from Re/Im to Amplitude/Phase
AnalyseResultType do_result_conversion(float real, float imag)
{
  AnalyseResultType result;
  float amplitude = imag * imag + real*real;
  result.Amplitude = (uint32_t)sqrt(amplitude);
  
  float phase = atan2(imag, real);
  phase = phase * (float)(PHASE_MULT * MAX_ANGLE / 2) / M_PI;
  result.Phase = (int16_t)(phase);
  
  return result;
}

//phase in deg (0-3599)
//calculate average phase value
//data - array of values
int16_t calculate_avr_phase(int16_t* data, uint16_t length)
{
  uint16_t i;
  uint8_t zero_cross_flag = 0;
  int32_t tmp_val = 0;
  
  //test for zero cross
  for (i = 0; i < length; i++)
  {
    if ((data[i] < (ZERO_ANGLE1 * PHASE_MULT)) || (data[i] > (ZERO_ANGLE2 * PHASE_MULT))) zero_cross_flag = 1;
  }
  
  if (zero_cross_flag == 0) //no zero cross 
  {
    //simple avarege calculation
    for (i = 0; i < length; i++) {tmp_val+= (int32_t)data[i];}
    tmp_val = tmp_val / length;
    return (uint16_t)tmp_val;
  }
  else
  {
    //remove zero crossing
    for (i = 0; i < length; i++) 
    {
      tmp_val+= (data[i] > HMAX_ANGLE * PHASE_MULT) ?  ((HMAX_ANGLE + MAX_ANGLE) * PHASE_MULT - data[i]) : (HMAX_ANGLE * PHASE_MULT - data[i]); //0->180, 359->181
    }
    tmp_val = tmp_val / length;//positive value
    //remove additional shift
    tmp_val = HMAX_ANGLE * PHASE_MULT - tmp_val;//can be negative
    if (tmp_val < 0) 
      tmp_val = MAX_ANGLE * PHASE_MULT + tmp_val;
    return tmp_val;
  }
}

//phase is deg * 10
int16_t calculate_corrected_phase(
  uint16_t raw_temperature, uint16_t amplitude, uint8_t apd_voltage, int16_t phase)
{
  return phase;
}


