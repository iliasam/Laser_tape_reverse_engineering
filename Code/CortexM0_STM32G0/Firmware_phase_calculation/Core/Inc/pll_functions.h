#include "stm32g0xx.h"
#include "i2c_functions.h"

#ifndef _PLL_FUNC_H
#define _PLL_FUNC_H

void configure_pll(void);


void pll_send_enable_output(void);

void pll_set_frequency_1(void);
void pll_set_frequency_2(void);
void pll_set_frequency_3(void);


#endif
