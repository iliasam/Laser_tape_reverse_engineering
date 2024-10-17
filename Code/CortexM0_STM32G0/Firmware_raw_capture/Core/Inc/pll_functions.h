#include "stm32g0xx.h"
#include "i2c_functions.h"

#ifndef _PLL_FUNC_H
#define _PLL_FUNC_H

#define MSNA_PLL_START_REG (26)
#define MSNB_PLL_START_REG (34)

void configure_pll(void);
void set_pll_coeff(uint32_t a, uint32_t b, uint32_t c, uint8_t config_reg);

void pll_change_freq(uint32_t pll_mult, uint32_t plla_coef, uint32_t pllb_coef, uint32_t pll_div);

void pll_send_disable_output(void);
void pll_send_enable_output(void);

void pll_set_frequency_1(void);
void pll_set_frequency_2(void);
void pll_set_frequency_3(void);
void pll_set_frequency_4(void);
void pll_set_frequency_5(void);

#endif
