#include "stdlib.h"
#include "distance_calc.h"
#include "analyse.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

#define DIST_MULT       32 //resulting distance is multiplied to this value

#define WAVE_L1         59036 //wavelength for 162.5Mhz in mm * 32
#define WAVE_L2         50095 //wavelength for 191.5Mhz in mm * 32
#define WAVE_L3         49578 //wavelength for 193.5Mhz in mm * 32

#define BRUTFOCRCE_STEPS    6

int32_t calculate_distance(int16_t phase, int32_t wavelength, int16_t N);
int16_t find_best_n(int16_t N1, int16_t Nadd);
int16_t calculate_pnum(int16_t phaseA, int16_t phaseB, int32_t wavelengthA, int32_t wavelengthB);
int16_t calculate_pnum_add(int16_t phaseA, int16_t phaseB, int32_t wavelengthA, int32_t wavelengthB);
int32_t dual_N_distance_calculation(int16_t phaseA, int16_t phaseB, int32_t wavelengthA,  int32_t wavelengthB);
int32_t brutforce_dist_calculation(int16_t phaseA, int16_t phaseB, int32_t wavelengthA,  int32_t wavelengthB, int16_t startN, int16_t stopN);

int32_t triple_dist_calculaton(int16_t phase1, int16_t phase2, int16_t phase3)
{
  int32_t result_dist = 0;
  
  if (phase_close_to_zero(phase1) && phase_close_to_zero(phase2) && phase_close_to_zero(phase3))
  {
    printf("Zero mode\r\n");
    int32_t dist1, dist2, dist3;
    //close to zero
    if (phase1 < (HMAX_ANGLE * PHASE_MULT))
      dist1 = calculate_distance(phase1, WAVE_L1, 0);
    else
      dist1 = calculate_distance(phase1, WAVE_L1, -1);
    
    if (phase2 < (HMAX_ANGLE * PHASE_MULT)) 
      dist2 = calculate_distance(phase2, WAVE_L2, 0);
    else
      dist2 = calculate_distance(phase2, WAVE_L2, -1);
    
    if (phase3 < (HMAX_ANGLE * PHASE_MULT))
      dist3 = calculate_distance(phase3, WAVE_L3, 0);
    else
      dist3 = calculate_distance(phase3, WAVE_L3, -1);
    
    result_dist = (dist1 + dist2 + dist3)/ (3 * DIST_MULT * 2);
    
    return result_dist;
  }
  
  int32_t coarse_dist = dual_N_distance_calculation(phase2, phase3, WAVE_L2,  WAVE_L3);
  //calculate bounds for period number for freq1
  int16_t startN = (int)(coarse_dist / WAVE_L1) - 3;
  if (startN < 0) startN = 0;
  int16_t stopN = startN + BRUTFOCRCE_STEPS;
  
  result_dist = brutforce_dist_calculation(phase1, phase2, WAVE_L1,  WAVE_L2, startN, stopN) / (DIST_MULT * 2);// /2- double wave path
  
  return result_dist;
}

int32_t brutforce_dist_calculation(int16_t phaseA, int16_t phaseB, int32_t wavelengthA,  int32_t wavelengthB, int16_t startN, int16_t stopN)
{
  int16_t NA, NB;
  int16_t min_NA = 0; 
  int16_t min_NB = 0;
  int16_t stopNB = stopN + 4;
  
  int32_t distA_table[BRUTFOCRCE_STEPS];
  int32_t distB_table[BRUTFOCRCE_STEPS + 4];
  int32_t cur_diff;
  int32_t min_diff = 1000000;
  int32_t tmp_dist_value;
  
  //fill distance tables
  tmp_dist_value = calculate_distance(phaseA, wavelengthA, startN);
  for (NA = 0; NA < BRUTFOCRCE_STEPS; NA++)
  {
    distA_table[NA] = tmp_dist_value;
    tmp_dist_value+= wavelengthA;
  }
  
  tmp_dist_value = calculate_distance(phaseB, wavelengthB, startN);
  for (NB = 0; NB < (BRUTFOCRCE_STEPS + 4); NB++)
  {
    distB_table[NB] = tmp_dist_value;
    tmp_dist_value+= wavelengthB;
  }
  
  //find mininum difference positions
  for (NA = startN; NA < stopN; NA++)
  {
    for (NB = startN; NB < stopNB; NB++)
    {
      cur_diff = abs(distA_table[NA - startN] - distB_table[NB - startN]);
      if (cur_diff < min_diff)
      {
        min_diff = cur_diff;
        min_NA = NA;
        min_NB = NB;
      }
    }
  }
  
  tmp_dist_value = (distA_table[min_NA - startN] + distB_table[min_NB - startN]) / 2;
  
  return tmp_dist_value;
}


//try to find coarse distance for two frequencies
//return distance in mm * DIST_MULT
int32_t dual_N_distance_calculation(int16_t phaseA, int16_t phaseB, int32_t wavelengthA,  int32_t wavelengthB)
{
  int16_t N_base = calculate_pnum(phaseA, phaseB, wavelengthA, wavelengthB);
  int16_t N_add = calculate_pnum_add(phaseA, phaseB, wavelengthA, wavelengthB);
  int16_t N_best =  find_best_n(N_base, N_add);

  return calculate_distance(phaseA, wavelengthA, N_best);
}


//n - number of periods
int32_t calculate_distance(int16_t phase, int32_t wavelength, int16_t N)
{
  int32_t result = wavelength * N + ((int32_t)phase * wavelength) / (MAX_ANGLE * PHASE_MULT);
  return result;
}

int16_t find_best_n(int16_t N1, int16_t Nadd)
{
  if (N1 == Nadd) return N1;
  if (Nadd > 40) return N1;
  if (N1 < 0) return Nadd;
  if (Nadd < 0) return N1;
  
  //worst cases
  if (N1 == 0) return Nadd;
  if (Nadd == 0) return N1;
  
  return N1;
}

//calculate number of periods
//return number of periods
int16_t calculate_pnum(int16_t phaseA, int16_t phaseB, int32_t wavelengthA, int32_t wavelengthB)
{
  int16_t tmp_result;
  tmp_result = (wavelengthB * phaseB - wavelengthA * phaseA) / ((wavelengthA - wavelengthB) * MAX_ANGLE * PHASE_MULT);
  return tmp_result;
}

//calculate number of periods
//return number of periods
int16_t calculate_pnum_add(int16_t phaseA, int16_t phaseB, int32_t wavelengthA, int32_t wavelengthB)
{
  int16_t tmp_result;
  tmp_result = (wavelengthB * phaseB - wavelengthA * phaseA + wavelengthB*(MAX_ANGLE * PHASE_MULT)) / ((wavelengthA - wavelengthB) * MAX_ANGLE * PHASE_MULT);
  return tmp_result;
}

