#include "stm32f0xx_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "pll_functions.h"
#include "measure_functions.h"
#include "delay_us_timer.h"
#include "distance_calc.h"
#include "uart_handler.h"
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

#define ENHANCED_APD_CALIBADION         1

/* Private variables ---------------------------------------------------------*/

AnalyseResultType result1;
AnalyseResultType result2;
AnalyseResultType result3;

extern uint16_t APD_temperature_raw ;//raw temperature value
extern float  APD_current_voltage;//value in volts
extern uint8_t  measure_enabled;//auto distance measurement enabled flag
extern volatile uint8_t capture_done;
extern volatile uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];//signal+reference points

volatile dma_state_type dma_state = DMA_NO_DATA;
uint8_t new_data_captured = 0;//new data ready to be processed

int16_t zero_phase1_calibration = 0;//phase value for zero distance
int16_t zero_phase2_calibration = 0;//phase value for zero distance
int16_t zero_phase3_calibration = 0;//phase value for zero distance

//signal+reference points
extern volatile uint16_t adc_capture_buffer0[ADC_CAPURE_BUF_LENGTH];
extern volatile uint16_t adc_capture_buffer1[ADC_CAPURE_BUF_LENGTH];

volatile uint16_t *capture_ptr = adc_capture_buffer0;
volatile uint16_t *process_ptr = adc_capture_buffer1;

int32_t dist_result_mm = 0;

//debug only - in us
volatile uint16_t delta_time = 0;

//float apd_saturation_voltage = APD_DEFAULT_SATURATIION_VOLT;
//float apd_min_voltage = APD_DEFAULT_SATURATIION_VOLT - APD_VOLTAGE_RANGE;

uint16_t apd_voltage_calib_buf[APD_VOLT_CALIB_LENGTH];

// Amlitude is decreased ~2 times if voltage is decreased for this value
uint8_t apd_voltage_decrease = APD_VOLT_DECREASE_DEFAULT_V;

uint8_t debug_freq_num = 0;

/* Private functions ---------------------------------------------------------*/
void enhanced_apd_calibration(void);
ErrorStatus calibration_set_voltage(void);
AnalyseResultType process_captured_data(uint16_t* captured_data);
void measure_swap_adc_buffers(void);

/* Private functions ---------------------------------------------------------*/

void measure_swap_adc_buffers(void)
{
  if (capture_ptr == adc_capture_buffer0)
  {
    capture_ptr = adc_capture_buffer1;
    process_ptr = adc_capture_buffer0;
  }
  else
  {
    capture_ptr = adc_capture_buffer0;
    process_ptr = adc_capture_buffer1;
  }
}

//Auto switch capture process
void auto_handle_capture(void)
{  
  if (new_data_captured == 1) 
    return;//it's not allowed to switch capture when previous data are not processed
  
  if (dma_state == DMA_NO_DATA)
  {    
    pll_set_frequency_1();//162.5 + 162.505
    dwt_delay(SWITCH_DELAY);
    measure_swap_adc_buffers();
    start_adc_capture(capture_ptr);
    dma_state = DMA_FREQ1_CAPTURING;
    new_data_captured = 1;//data3 ready
  }
  else if (dma_state == DMA_FREQ1_DONE)
  {
    //ready to switch to freq2
    pll_set_frequency_2();//191.5 + 191.505
    dwt_delay(SWITCH_DELAY);
    measure_swap_adc_buffers();    
    start_adc_capture(capture_ptr);
    dma_state = DMA_FREQ2_CAPTURING;
    new_data_captured = 1;//data1 ready
  }
  else if (dma_state == DMA_FREQ2_DONE)
  {
    //ready to switch to freq3
    pll_set_frequency_3();//193.5 + 193.505
    dwt_delay(SWITCH_DELAY);
    measure_swap_adc_buffers();
    start_adc_capture(capture_ptr);
    dma_state = DMA_FREQ3_CAPTURING;
    new_data_captured = 1;//data2 ready
  }
  else if (dma_state == DMA_FREQ3_DONE)
  {
    //all data captured now
    dma_state = DMA_NO_DATA;
    
    capture_do_single_adc_measurements();//measure temperature
    auto_switch_apd_voltage(result1.Amplitude);//if auto switch enabled, manual switching is not working
  }
}

void auto_handle_data_processing(void)
{
  static uint16_t old_dwt_value = 0;
  static char result_str[64];
  uint16_t result_length;

  if (new_data_captured == 0) 
    return; //nothing to process
  
  if ((dma_state == DMA_FREQ1_CAPTURING) || (dma_state == DMA_FREQ1_DONE))
  {
    //debug functions
    uint16_t cur_dwt_value = get_dwt_value();
    delta_time = cur_dwt_value - old_dwt_value;
    old_dwt_value = cur_dwt_value;
  
    debug_freq_num = 3;
    result3 = process_captured_data((uint16_t*)process_ptr);
    do_distance_calculation();
    
#ifdef FAST_CAPTURE
  //Less information
  result_length = sprintf(
      result_str, "%05d;%04d\r\n", dist_result_mm, result1.Amplitude);
#else
  result_length = sprintf(
      result_str, "DIST;%05d;AMP;%04d;TEMP;%04d;VOLT;%03d\r\n", 
      dist_result_mm, result1.Amplitude, APD_temperature_raw, (uint8_t)APD_current_voltage);
#endif
    
    uart_dma_start_tx((uint8_t*)result_str, result_length);//attention - new transmission interrupts previous one. 
    new_data_captured = 0;
  }
  else if ((dma_state == DMA_FREQ2_CAPTURING) || (dma_state == DMA_FREQ2_DONE)) 
  {
    debug_freq_num = 1;
    result1 = process_captured_data((uint16_t*)process_ptr);
    new_data_captured = 0;
  }
  else if ((dma_state == DMA_FREQ3_CAPTURING) || (dma_state == DMA_FREQ3_DONE)) 
  {
    debug_freq_num = 2;
    result2 = process_captured_data((uint16_t*)process_ptr);
    new_data_captured = 0;
  }
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
}

ErrorStatus do_phase_calibration(void)
{
  int16_t tmp_phase1 = 0;
  int16_t tmp_phase2 = 0;
  int16_t tmp_phase3 = 0;
  
  //short_beep();
  
  printf("Calib Start U85\r\n");
  enable_laser();
  measure_enabled = 1;
  
  enhanced_apd_calibration();
  
  delay_ms(400);
  capture_do_single_adc_measurements();//measure temperature
  auto_switch_apd_voltage(result1.Amplitude);

  //set freq1
  pll_set_frequency_1();//162.5 + 162.505
  delay_ms(100);
  
  //try to find best voltage for calibration
  if (calibration_set_voltage() == ERROR) 
    return ERROR;
  
  if (single_freq_calibration(&tmp_phase1) == ERROR) 
    return ERROR;
  printf("Zero Phase 1: %d\r\n", tmp_phase1);
  
  //set freq2
  pll_set_frequency_2();//191.5 + 191.505
  delay_ms(100);
  if (single_freq_calibration(&tmp_phase2) == ERROR) 
    return ERROR;
  printf("Zero Phase 2: %d\r\n", tmp_phase2);
  
  //set freq3
  pll_set_frequency_3();//193.5 + 193.505
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







//phase measurement for single freqency
//used for calibration
AnalyseResultType do_capture(void)
{
  dma_state = DMA_NO_DATA;
  new_data_captured = 0;
  
#if (REPEAT_NUMBER == 1)
  
  start_adc_capture((uint16_t*)adc_capture_buffer0);
  while(capture_done == 0) {}
  return process_captured_data((uint16_t*)adc_capture_buffer0);

#else
  
  AnalyseResultType main_result = {0,0};
  AnalyseResultType signal_result = {0,0};
  AnalyseResultType reference_result = {0,0};
  uint8_t i;
  int16_t phase_buffer[REPEAT_NUMBER];
  uint32_t amplitude_summ = 0;
  int16_t tmp_phase = 0;
  
  for (i = 0; i < REPEAT_NUMBER; i++)
  {
    start_adc_capture((uint16_t*)adc_capture_buffer0);
    while(capture_done == 0) {}
    signal_result = goertzel_analyse((uint16_t*)&adc_capture_buffer0[0]);//signal
    reference_result = goertzel_analyse((uint16_t*)&adc_capture_buffer0[1]);//reference
    
    tmp_phase = signal_result.Phase - reference_result.Phase;//difference between signal and reference phase
    if (tmp_phase < 0) 
      tmp_phase = MAX_ANGLE * PHASE_MULT + tmp_phase;
    
    amplitude_summ+= (uint32_t)signal_result.Amplitude;
    phase_buffer[i] = tmp_phase;
  }
  
  main_result.Amplitude = (uint16_t)(amplitude_summ / REPEAT_NUMBER);
  main_result.Phase = calculate_avr_phase(phase_buffer, REPEAT_NUMBER);
  main_result.RawPhase = main_result.Phase;
  
  main_result.Phase = calculate_true_phase(
    APD_temperature_raw, main_result.Amplitude, (uint8_t)APD_current_voltage, main_result.Phase);
  //phase correction can produce zero crossing
  //phase < 0 or > 360
  if (main_result.Phase < 0) 
    main_result.Phase = MAX_ANGLE * PHASE_MULT + main_result.Phase;
  else if (main_result.Phase > (MAX_ANGLE * PHASE_MULT)) 
    main_result.Phase = main_result.Phase - MAX_ANGLE * PHASE_MULT;
  
  return main_result;
#endif
}

AnalyseResultType process_captured_data(uint16_t* captured_data)
{
  AnalyseResultType main_result = {0,0};
  AnalyseResultType signal_result = {0,0};
  AnalyseResultType reference_result = {0,0};
  int16_t tmp_phase = 0;
  
  signal_result    = goertzel_analyse(&captured_data[0]);//signal
  reference_result = goertzel_analyse(&captured_data[1]);//reference
  tmp_phase = signal_result.Phase - reference_result.Phase;//difference between signal and reference phase
  if (tmp_phase < 0) 
    tmp_phase = MAX_ANGLE * PHASE_MULT + tmp_phase;
  main_result.Phase = tmp_phase;
  main_result.Amplitude = signal_result.Amplitude;
  main_result.RawPhase = main_result.Phase;
  
  //calculate correction
  main_result.Phase = calculate_true_phase(
    APD_temperature_raw, main_result.Amplitude, (uint8_t)APD_current_voltage, main_result.Phase);
  if (main_result.Phase < 0) 
    main_result.Phase = MAX_ANGLE * PHASE_MULT + main_result.Phase;
  else if (main_result.Phase > (MAX_ANGLE * PHASE_MULT)) 
    main_result.Phase = main_result.Phase - MAX_ANGLE * PHASE_MULT;
  
  return main_result;
}

// Try to find APD saturation voltage
void enhanced_apd_calibration(void)
{
#if (ENHANCED_APD_CALIBADION)
  
  float apd_calibration_voltage = APD_START_CALIB_VOLTAGE;
  AnalyseResultType tmp_result;
  uint8_t signal_detected_flag = 0;
  
  delay_ms(300);
  set_apd_voltage(apd_calibration_voltage);
  delay_ms(200);
  
  uint8_t step = 0;
  while (apd_calibration_voltage < APD_STOP_CALIB_VOLTAGE)
  {
    tmp_result = do_capture();//measure signal
    apd_voltage_calib_buf[step] = tmp_result.Amplitude;
    
    if (tmp_result.Amplitude > ENHANCED_CALIBADION_START_AMPL)
      signal_detected_flag = 1; //Signal detected
    
    if ((signal_detected_flag == 1) && (tmp_result.Amplitude < ENHANCED_CALIBADION_STOP_AMPL))
    {
      //Get APD voltage range
      //apd_saturation_voltage = apd_calibration_voltage - 5.0f;
     // apd_min_voltage = apd_saturation_voltage - APD_VOLTAGE_RANGE;
      if (step > 1)
        step--;
      break;
    }
    
    step++;
    apd_calibration_voltage += APD_VOLT_CALIB_STEP_V;
    set_apd_voltage(apd_calibration_voltage);
    delay_ms(200);
  }
  
  apd_voltage_decrease = APD_VOLT_DECREASE_DEFAULT_V;
  if (step > 1)
  {
    uint16_t threshold_amplitude = tmp_result.Amplitude / 2;
    for (uint8_t i = 0; i < step; i++)
    {
      if (apd_voltage_calib_buf[i] > threshold_amplitude)
      {
        apd_voltage_decrease =
           (uint8_t)(APD_STOP_CALIB_VOLTAGE - 
                    (APD_START_CALIB_VOLTAGE + APD_VOLT_CALIB_STEP_V * i));
        if (apd_voltage_decrease > 50) //error
          apd_voltage_decrease = APD_VOLT_DECREASE_DEFAULT_V;
        break;
      }
    }
  }
  
  //printf("APD Saturation Voltage: %.1f\r\n", apd_saturation_voltage);
  
#endif
}

//write calibratio data to flash
void write_data_to_flash(int16_t calib_phase1, int16_t calib_phase2, int16_t calib_phase3)
{
  uint32_t start_address = 0x8000000 + 1024*31;
  FLASH_Unlock();
  FLASH_ErasePage(start_address); //erase page 31
  FLASH_ProgramHalfWord(start_address, 0x1234); //data present key
  FLASH_ProgramHalfWord(start_address+2, (uint16_t)calib_phase1); //phase
  FLASH_ProgramHalfWord(start_address+4, (uint16_t)calib_phase2); //phase
  FLASH_ProgramHalfWord(start_address+6, (uint16_t)calib_phase3); //phase
  //FLASH_ProgramHalfWord(start_address+8, (uint16_t)apd_saturation_voltage);
  FLASH_ProgramHalfWord(start_address+10, (uint16_t)apd_voltage_decrease);
  
  FLASH_Lock();
}

//read calibration data
void read_calib_data_from_flash(void)
{
  uint32_t start_address = 0x8000000 + 1024*31;
  uint16_t data_key = (*(__IO uint16_t*)start_address);
  if (data_key != 0x1234)
    return; //no data in flash
  
  zero_phase1_calibration = (*(__IO int16_t*)(start_address + 2));
  zero_phase2_calibration = (*(__IO int16_t*)(start_address + 4));
  zero_phase3_calibration = (*(__IO int16_t*)(start_address + 6));
  
  uint16_t tmp_value = (*(__IO uint16_t*)(start_address + 8));//voltage
  if ((tmp_value < 50) || (tmp_value > 150)) //apd voltage
  {}
  else
  {
    //apd_saturation_voltage = (float)tmp_value;
    //apd_min_voltage = apd_saturation_voltage - APD_VOLTAGE_RANGE;
  }
  
  uint16_t tmp_value2 = (*(__IO int16_t*)(start_address + 10));
  if (tmp_value2 < 50)
    apd_voltage_decrease  = (uint8_t)tmp_value2;
}
