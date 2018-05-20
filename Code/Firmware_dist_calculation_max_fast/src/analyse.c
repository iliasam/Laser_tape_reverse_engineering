#include "stm32f10x_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "math.h"
#include "analyse.h"
#include "stdlib.h"

#ifndef M_PI
#define M_PI            3.1415926535 
#endif

#define OMEGA           ((2.0 * M_PI * K_COEF) / POINTS_TO_SAMPLE)

#define INT_COEF        1024.0f

#define SCALING         ((float)POINTS_TO_SAMPLE / 2.0f)

//Static buffers
int32_t sin_buf[POINTS_TO_SAMPLE];
int32_t cos_buf[POINTS_TO_SAMPLE];

extern float  APD_temperature;//temperature value in deg

void init_goertzel(void)
{
  uint16_t i;
  
  for (i=0; i<POINTS_TO_SAMPLE; i++)
  {
    sin_buf[i] = (int32_t)(sin(i * OMEGA) * INT_COEF);
    cos_buf[i] = (int32_t)(cos(i * OMEGA) * INT_COEF);
  }
}

//data goes like data[used] + data[not used] + data[used] ...
AnalyseResultType goertzel_analyse(uint16_t* data)
{
  uint16_t i;
  int32_t real = 0.0;
  int32_t imag = 0.0;
  
  AnalyseResultType result;
  
  for(i=0; i<POINTS_TO_SAMPLE; i++)
  {
    real+= sin_buf[i] * (int32_t)data[i * 2];
    imag+= cos_buf[i] * (int32_t)data[i * 2];
  }
  
  real = real / (int32_t)SCALING;
  imag = imag / (int32_t)SCALING;
  
  result = do_result_conversion((float)real, (float)imag);
  return result;
}

//convert from Re/Im to Amplitude/Phase
AnalyseResultType do_result_conversion(float real, float imag)
{
  AnalyseResultType result;
  
  float amplitude = imag * imag + real * real;
  amplitude = sqrtf(amplitude);
  result.Amplitude = (uint16_t)(amplitude / INT_COEF);
  
  float phase = atan2f(imag, real);
  phase = phase * (float)(PHASE_MULT * HMAX_ANGLE) / M_PI;
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
    //if (phase_close_to_zero(data[i]) == 1) zero_cross_flag = 1;
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

// Calculate phase_correction value
#ifdef MODULE_701A

int16_t calculate_correction(uint16_t raw_temperature, uint16_t amplitude, uint8_t apd_voltage)
{
  //temperature compensation
  float correction = 0.226974f * APD_temperature; //it is bad to use APD_temperature here
  correction+= -0.0049827f * APD_temperature * APD_temperature;
  return (int16_t)(-correction * PHASE_MULT);
}

#else

int16_t calculate_correction(uint16_t raw_temperature, uint16_t amplitude, uint8_t apd_voltage)
{
  int32_t tmp_value = 0;
  double tmp_value_d = 0.0;
  
  if (apd_voltage < (uint8_t)(APD_LOW_VOLTAGE + 2.0f))
  {
    //temperature compensation
    tmp_value = (int32_t)raw_temperature * (int32_t)(-7* PHASE_MULT) / 1024;
  }
  else
  {
    //temperature compensation
    tmp_value_d = -1.0 * (double)PHASE_MULT * exp( ((double)raw_temperature - 720.0) / 280.0);
    tmp_value = (int32_t)tmp_value_d;
    //amplitude compensation
    tmp_value = tmp_value-((int32_t)amplitude/(-1000/PHASE_MULT) + 2*PHASE_MULT);
    //Voltage correction
    tmp_value-= 55;//PHASE_MULT used
    //TODO - use CORR_360_DEG for MAX_ANGLE != 360
  }
  return (int16_t)tmp_value_d;
}

#endif

//return 1 if phase is close to zero
uint8_t phase_close_to_zero(int16_t phase)
{
  if (phase < (ZERO_ANGLE1 * PHASE_MULT)) return 1;
  if (phase > (ZERO_ANGLE2 * PHASE_MULT)) return 1;
  return 0;
}




