#include "stm32g0xx_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "pll_functions.h"
#include "measure_functions.h"
#include "delay_us_timer.h"
#include "distance_calc.h"
#include "stm32g0xx_ll_flash.h"
#include "stdlib.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/

#define CALIBRATION_REPEAT_NUMBER       64 //number of averaging points for calibration - for single freqency
#define REPEAT_NUMBER                   1 //number of averaging points
#define SWITCH_DELAY                    300 //time in uS to switch frequency 

//Starting amplitude, ADC points
#define ENHANCED_CALIBADION_START_AMPL  200

//Stop amplitude, ADC points
#define ENHANCED_CALIBADION_STOP_AMPL   5

#define STM32G0_PAGE_SIZE_BYTES         2048
#define NVRAM_PAGE                      17

//#define ENHANCED_APD_CALIBADION

/* Private variables ---------------------------------------------------------*/

AnalyseResultType result1;
AnalyseResultType result2;
AnalyseResultType result3;

extern uint16_t APD_temperature_raw ;//raw temperature value
extern float  APD_current_voltage;//value in volts
extern uint8_t  measure_enabled;//auto distance measurement enabled flag
extern volatile uint8_t capture_done;
extern volatile uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];//signal+reference points

int16_t zero_phase1_calibration = 0;//phase value for zero distance
int16_t zero_phase2_calibration = 0;//phase value for zero distance
int16_t zero_phase3_calibration = 0;//phase value for zero distance

int32_t dist_result_mm = 0;

float apd_saturation_voltage = APD_DEFAULT_SATURATIION_VOLT;
float apd_min_voltage = APD_DEFAULT_SATURATIION_VOLT - APD_VOLTAGE_RANGE;

/* Private functions ---------------------------------------------------------*/
void enhanced_apd_calibration(void);
ErrorStatus calibration_set_voltage(void);

/* Private functions ---------------------------------------------------------*/

ErrorStatus do_phase_calibration(void)
{
  int16_t tmp_phase1 = 0;
  int16_t tmp_phase2 = 0;
  int16_t tmp_phase3 = 0;
  
  //short_beep();
  
  printf("Calib Start B2A\r\n");
  enable_laser();
  measure_enabled = 1;
  
  enhanced_apd_calibration();
  
  delay_ms(400);
  capture_do_single_adc_measurements();//measure temperature
  auto_switch_apd_voltage(result1.Amplitude);

  //set freq1
  pll_set_frequency_1();//162.0
  delay_ms(100);
  
  //try to find best voltage for calibration
  if (calibration_set_voltage() == ERROR) 
    return ERROR;
  
  if (single_freq_calibration(&tmp_phase1) == ERROR) 
    return ERROR;
  printf("Zero Phase 1: %d\r\n", tmp_phase1);
  
  //set freq2
  pll_set_frequency_2();
  delay_ms(100);
  if (single_freq_calibration(&tmp_phase2) == ERROR) 
    return ERROR;
  printf("Zero Phase 2: %d\r\n", tmp_phase2);
  
  //set freq3
  pll_set_frequency_3();
  delay_ms(100);
  if (single_freq_calibration(&tmp_phase3) == ERROR) 
    return ERROR;
  printf("Zero Phase 3: %d\r\n", tmp_phase3);
  
  zero_phase1_calibration = tmp_phase1;
  zero_phase2_calibration = tmp_phase2;
  zero_phase3_calibration = tmp_phase3;
  
  write_data_to_flash(tmp_phase1, tmp_phase2, tmp_phase3);
  
  //short_beep();
  
  printf("Calib Done\r\n");
  return SUCCESS;
}

//try to find best voltage for calibration
ErrorStatus calibration_set_voltage(void)
{
  AnalyseResultType tmp_result;
  tmp_result = do_capture();//measure signal

  if (tmp_result.Amplitude < 5) //amplitude too low
  {
    printf("Calib FAIL: Low Signal\r\n");
    return ERROR;
  } 
  
  printf("Calib voltage:%d\r\n", (uint8_t)APD_current_voltage);
  
  return SUCCESS;
}

//calibration measurement for single frequency
//result_phase - result value will be written at this pointer
ErrorStatus single_freq_calibration(int16_t* result_phase)
{
  uint8_t i;
  AnalyseResultType tmp_result;
  int16_t phase_array[CALIBRATION_REPEAT_NUMBER];
  
  //do main measurements
  for (i = 0; i < CALIBRATION_REPEAT_NUMBER; i++)
  {
    tmp_result = do_capture();//measure signal
    
    if (tmp_result.Amplitude < 50)
    {
      printf("Calib FAIL: Low Signal\r\n");
      return ERROR;
    }
    phase_array[i] = tmp_result.Phase;
  }
  //calculate average phase value
  int16_t calib_result = calculate_avr_phase(phase_array, CALIBRATION_REPEAT_NUMBER);
  *result_phase = calib_result;
  
  return SUCCESS;
}

void do_triple_phase_measurement(void)
{
    pll_set_frequency_1();
    dwt_delay(SWITCH_DELAY);
    result1 = do_capture();
    
    pll_set_frequency_2();
    dwt_delay(SWITCH_DELAY);
    result2 = do_capture();
    
    pll_set_frequency_3();
    dwt_delay(SWITCH_DELAY);
    result3 = do_capture();
}

void do_distance_calculation(void)
{
  //subtract zero phase offset
  int16_t tmp_phase1 = result1.Phase - zero_phase1_calibration;
  if (tmp_phase1 < 0) 
    tmp_phase1 = MAX_ANGLE * PHASE_MULT + tmp_phase1;
  
  int16_t tmp_phase2 = result2.Phase - zero_phase2_calibration;
  if (tmp_phase2 < 0) 
    tmp_phase2 = MAX_ANGLE * PHASE_MULT + tmp_phase2;
  
  int16_t tmp_phase3 = result3.Phase - zero_phase3_calibration;
  if (tmp_phase3 < 0) 
    tmp_phase3 = MAX_ANGLE * PHASE_MULT + tmp_phase3;
  
  dist_result_mm = triple_dist_calculaton(tmp_phase1, tmp_phase2, tmp_phase3);
  uint16_t tmp_volt = (uint16_t)APD_current_voltage;
  printf("DIST;%05d;AMP;%04d;TEMP;%04d;VOLT;%03d\r\n", 
    dist_result_mm, result1.Amplitude, APD_temperature_raw, tmp_volt);
}

//phase measurement for single freqency
AnalyseResultType do_capture(void)
{
  AnalyseResultType main_result = {0,0};
  AnalyseResultType signal_result = {0,0};
  AnalyseResultType reference_result = {0,0};

  int16_t tmp_phase = 0;
  
#if (REPEAT_NUMBER == 1)
  
  start_adc_capture();
  while(capture_done == 0) {}
  signal_result = goertzel_analyse((uint16_t*)&adc_capture_buffer[0]);//signal
  reference_result = goertzel_analyse((uint16_t*)&adc_capture_buffer[1]);//reference
  //difference between signal and reference phase
  tmp_phase =  reference_result.Phase - signal_result.Phase;
  if (tmp_phase < 0) 
    tmp_phase = MAX_ANGLE * PHASE_MULT + tmp_phase;
  main_result.Phase = tmp_phase;
  main_result.Amplitude = signal_result.Amplitude;

#else
  uint8_t i;
  int16_t phase_buffer[REPEAT_NUMBER];
  uint32_t amplitude_summ = 0;
  
  for (i = 0; i < REPEAT_NUMBER; i++)
  {
    start_adc_capture();
    while(capture_done == 0) {}
    signal_result = goertzel_analyse((uint16_t*)&adc_capture_buffer[0]);//signal
    reference_result = goertzel_analyse((uint16_t*)&adc_capture_buffer[1]);//reference
    
    //difference between signal and reference phase
    tmp_phase = reference_result.Phase - signal_result.Phase;
    if (tmp_phase < 0) 
      tmp_phase = MAX_ANGLE * PHASE_MULT + tmp_phase;
    
    amplitude_summ+= (uint32_t)signal_result.Amplitude;
    phase_buffer[i] = tmp_phase;
  }
  
  main_result.Amplitude = (uint16_t)(amplitude_summ / REPEAT_NUMBER);
  main_result.Phase = calculate_avr_phase(phase_buffer, REPEAT_NUMBER);
  
#endif  

  main_result.Phase = calculate_true_phase(
    APD_temperature_raw, main_result.Amplitude, (uint8_t)APD_current_voltage, main_result.Phase);
  //phase corretion can produce zero crossing
  //phase < 0 or > 360
  if (main_result.Phase < 0.0) 
    main_result.Phase = MAX_ANGLE * PHASE_MULT + main_result.Phase;
  else if (main_result.Phase > (MAX_ANGLE * PHASE_MULT)) 
    main_result.Phase = main_result.Phase - MAX_ANGLE * PHASE_MULT;
  
  return main_result;
}

// Try to find APD saturation voltage
void enhanced_apd_calibration(void)
{
#ifdef ENHANCED_APD_CALIBADION
  
  float apd_calibration_voltage = APD_START_CALIB_VOLTAGE;
  AnalyseResultType tmp_result;
  uint8_t signal_detected_flag = 0;
  
  delay_ms(300);
  
  set_apd_voltage(apd_calibration_voltage);
  delay_ms(200);
  
  while (apd_calibration_voltage < APD_STOP_CALIB_VOLTAGE)
  {
    tmp_result = do_capture();//measure signal
    if (tmp_result.Amplitude > ENHANCED_CALIBADION_START_AMPL)
      signal_detected_flag = 1; //Signal detected
    
    if ((signal_detected_flag == 1) && (tmp_result.Amplitude < ENHANCED_CALIBADION_STOP_AMPL))
    {
      //Get APD voltage range
      apd_saturation_voltage = apd_calibration_voltage - 5.0f;
      apd_min_voltage = apd_saturation_voltage - APD_VOLTAGE_RANGE;
      break;
    }
    
    apd_calibration_voltage += 1.0f;
    set_apd_voltage(apd_calibration_voltage);
    delay_ms(200);
  }
  printf("APD Saturation Voltage: %.1f\r\n", apd_saturation_voltage);
  
#endif
}

//write calibratio data to flash
void write_data_to_flash(int16_t calib_phase1, int16_t calib_phase2, int16_t calib_phase3)
{
  return;
  uint32_t start_address = 0x8000000 + STM32G0_PAGE_SIZE_BYTES * NVRAM_PAGE;
  LL_Flash_Unlock();
  
  LL_Flash_PageErase(NVRAM_PAGE);
  LL_FLASH_Program(ProgaraType_DATA16, start_address, 0x1234);
  LL_FLASH_Program(ProgaraType_DATA16, start_address+2, (uint16_t)calib_phase1);
  LL_FLASH_Program(ProgaraType_DATA16, start_address+4, (uint16_t)calib_phase2);
  LL_FLASH_Program(ProgaraType_DATA16, start_address+6, (uint16_t)calib_phase3);
  LL_FLASH_Program(ProgaraType_DATA16, start_address+8, (uint16_t)apd_saturation_voltage);
  /*
  LL_FLASH_Program_HalfWord(start_address, 0x1234); //data present key
  LL_FLASH_Program_HalfWord(start_address+2, (uint16_t)calib_phase1); //phase
  LL_FLASH_Program_HalfWord(start_address+4, (uint16_t)calib_phase2); //phase
  LL_FLASH_Program_HalfWord(start_address+6, (uint16_t)calib_phase3); //phase
  LL_FLASH_Program_HalfWord(start_address+8, (uint16_t)apd_saturation_voltage);
  */
  LL_Flash_Lock();
  
}

//read calibration data
void read_calib_data_from_flash(void)
{
  uint32_t start_address = 0x8000000 + STM32G0_PAGE_SIZE_BYTES * NVRAM_PAGE;
  uint16_t data_key = (*(__IO uint16_t*)start_address);
  if (data_key != 0x1234)
    return; //no data in flash
  
  zero_phase1_calibration = (*(__IO int16_t*)(start_address + 2));
  zero_phase2_calibration = (*(__IO int16_t*)(start_address + 4));
  zero_phase3_calibration = (*(__IO int16_t*)(start_address + 6));
  
  uint16_t tmp_value = (*(__IO uint16_t*)(start_address + 8));//voltage
  if ((tmp_value < 50) || (tmp_value > 150)) //apd voltage
    return;
  apd_saturation_voltage = (float)tmp_value;
  apd_min_voltage = apd_saturation_voltage - APD_VOLTAGE_RANGE;
}
