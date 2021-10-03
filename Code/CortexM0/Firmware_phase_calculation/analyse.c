#include "stm32f0xx_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "math.h"
#include "analyse.h"
#include "stdlib.h"

#ifndef M_PI
#define M_PI            3.1415926535f
#endif

extern float  APD_temperature;//temperature value in deg




#define OMEGA ((2.0 * M_PI * K_COEF) / POINTS_TO_SAMPLE)

float sin_coef = 0.0;
float cos_coef = 0.0;
float g_coef = 0.0;//goertzel coefficient

float calc_phase_offset_small(float a, float b, float start, float stop, float step);
float calc_phase_offset(float a, float b);


void init_goertzel(void)
{
  sin_coef = sin(OMEGA);
  cos_coef = cos(OMEGA);
  g_coef = 2.0 * cos_coef;
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
  
  for(i=0; i<POINTS_TO_SAMPLE; i++)
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
  for (i=0; i < length; i++)
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
    if (tmp_val < 0) tmp_val = MAX_ANGLE * PHASE_MULT + tmp_val;
    return tmp_val;
  }
}



//phase is deg * 10
int16_t calculate_correction(
  uint16_t raw_temperature, uint16_t amplitude, uint8_t apd_voltage, uint16_t phase)
{
  //temperature compensation
  //it is bad to use APD_temperature here
  /// correction -  deg
  float correction_t = 0.226974f * APD_temperature; 
  correction_t+= -0.0049827f * APD_temperature * APD_temperature;
  
  float amp_corr_deg = 0.03f * amplitude - 9.5f;//deg
  float amp_corr_rad = amp_corr_deg * M_PI / 180.0f;
  
  float phase_rad = phase * 0.1 * M_PI / 180.0f;
  
  //Corrected phase, not offset
  float corr_rad = calc_phase_offset(amp_corr_rad, phase_rad);
  float corr_deg = corr_rad * 180.0f / M_PI;
  
  corr_deg = corr_deg - correction_t;
  
  return (int16_t)(corr_deg * PHASE_MULT);
}

//equation sin(x)=(x-b)/a
float calc_phase_offset_small(float a, float b, float start, float stop, float step)
{
  float min_error = 1;
  float best_x = start;
  
  for (float x = start; x < stop; x += step)
  {
    //b+a*sin(x)-x
    float error = a * sinf(x) + b - x;
    error = fabs(error);
    if (error < min_error)
    {
      min_error = error;
      best_x = x;
    }
  }
  
  return best_x;
}

//a is amplitude corection
//b is measured phase
//result is true phase
float calc_phase_offset(float a, float b)
{
  float start = -a + b - 0.1f;
  float stop = a + b + 0.1f;
  
  float best_x = calc_phase_offset_small(a, b, start, stop, 0.1f);
  start = best_x - 0.2f;
  stop = best_x + 0.2f;
  best_x = calc_phase_offset_small(a, b, start, stop, 0.01f);
  start = best_x - 0.02f;
  stop = best_x + 0.02f;
  best_x = calc_phase_offset_small(a, b, start, stop, 0.002f);
  
  return best_x;
}


