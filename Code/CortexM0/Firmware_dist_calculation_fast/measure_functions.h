#include "stm32f0xx.h"

#ifndef _MEASURE_FUNCTIONS_H
#define _MEASURE_FUNCTIONS_H

ErrorStatus single_freq_calibration(int16_t* calibration_result);
ErrorStatus do_phase_calibration(void);

void do_triple_phase_measurement(void);
AnalyseResultType do_capture(void);
void do_distance_calculation(void);

void write_data_to_flash(int16_t calib_phase1, int16_t calib_phase2, int16_t calib_phase3);
void read_calib_data_from_flash(void);


#endif
