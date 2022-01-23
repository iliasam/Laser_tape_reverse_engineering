/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "math.h"
#include "analyse.h"
#include "analyse_table.h" //generated with python script, constants must be coped to that script before running it
#include "stdlib.h"

/* Private typedef -----------------------------------------------------------*/
#ifndef M_PI
#define M_PI            3.1415926535f
#endif

#define INT_COEF        1024.0f
#define SCALING         ((float)POINTS_TO_SAMPLE / 2.0f)
#define OMEGA           ((2.0f * M_PI * K_COEF) / POINTS_TO_SAMPLE)

int debug_raw_freq = 0;
extern uint8_t debug_freq_num;

/* Private variables ---------------------------------------------------------*/
extern float  APD_temperature_deg;//temperature value in deg

void init_goertzel(void)
{
}

//data goes like data[used] + data[not used] + data[used] ...
AnalyseResultType goertzel_analyse(uint16_t* data)
{
  uint16_t i;
  int32_t real = 0; //integers!
  int32_t imag = 0;
  
  AnalyseResultType result;
  
  for(i=0; i < POINTS_TO_SAMPLE; i++)
  {
    real+= analyse_sin_table[i] * (int32_t)data[i * 2];
    imag+= analyse_cos_table[i] * (int32_t)data[i * 2];
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
  for (i = 0; i < length; i++)
  {
    if ((data[i] < (ZERO_ANGLE1 * PHASE_MULT)) || (data[i] > (ZERO_ANGLE2 * PHASE_MULT))) 
      zero_cross_flag = 1;
  }
  
  if (zero_cross_flag == 0) //no zero cross 
  {
    //simple avarege calculation
    for (i = 0; i < length; i++) 
      tmp_val+= (int32_t)data[i];
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
//take "phase" and calculate corrected phase, which would be returned
int16_t calculate_true_phase(
  uint16_t raw_temperature, uint16_t amplitude, uint8_t apd_voltage, uint16_t phase)
{
  //temperature compensation
  //it is bad to use APD_temperature_deg here
  // correction -  deg
  float correction_t = 0.226974f * APD_temperature_deg; 
  correction_t+= -0.0049827f * APD_temperature_deg * APD_temperature_deg;
  
  if (debug_freq_num == 1)
  {
    debug_raw_freq = (int)(phase / 10);
  }
  
  float corr_deg = (float)phase / (float)PHASE_MULT - correction_t;
  
  return (int16_t)(corr_deg * PHASE_MULT);
}

//return 1 if phase is close to zero
uint8_t phase_close_to_zero(int16_t phase)
{
  if (phase < (ZERO_ANGLE1 * PHASE_MULT)) 
    return 1;
  if (phase > (ZERO_ANGLE2 * PHASE_MULT)) 
    return 1;
  return 0;
}