#include "stm32f10x.h"
#include "analyse.h"

#ifndef _MEASURE_FUNCTIONS_H
#define _MEASURE_FUNCTIONS_H

typedef enum
{
  DMA_NO_DATA = 0,
  DMA_FREQ1_CAPTURING,
  DMA_FREQ1_DONE,
  
  DMA_FREQ2_CAPTURING,
  DMA_FREQ2_DONE,
  
  DMA_FREQ3_CAPTURING,
  DMA_FREQ3_DONE,
  
} dma_state_type;


void auto_switch_apd_voltage(uint16_t current_amplitude);

ErrorStatus single_freq_calibration(int16_t* calibration_result);
ErrorStatus do_phase_calibration(void);


AnalyseResultType do_capture(void);
void do_distance_calculation(void);

void write_data_to_flash(int16_t calib_phase1, int16_t calib_phase2, int16_t calib_phase3);
void read_calib_data_from_flash(void);

void auto_handle_capture(void);
void auto_handle_data_processing(void);


#endif
