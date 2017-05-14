#include "stm32f10x.h"

#ifndef _PLL_FUNC_H
#define _PLL_FUNC_H

#define MSNA_PLL_START_REG (26)
#define MSNB_PLL_START_REG (34)

void configure_pll(void);
void set_pll_coeff(uint32_t a, uint32_t b, uint32_t c, uint8_t config_reg);

#endif
