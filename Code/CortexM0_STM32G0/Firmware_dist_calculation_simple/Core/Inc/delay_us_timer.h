#ifndef __DELAY_US_TIMER
#define __DELAY_US_TIMER

#include "stdint.h"

#define F_CPU 24000000



void dwt_init(void);
void dwt_delay(uint16_t us);
uint16_t get_dwt_value(void);

#endif

