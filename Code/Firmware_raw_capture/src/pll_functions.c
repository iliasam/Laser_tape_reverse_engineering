#include "i2c_functions.h"
#include "pll_functions.h"
#include "stm32f10x_it.h"
#include "config_periph.h"

void PLL_send_enable_output(void);
void PLL_send_disable_output(void);
void PLL_send_fanout_enable(void);
void PLL_send_reset(void);
void PLL_send_data1(void);

const uint8_t pll_data_array_dis_output[2] = {3, 0xff};//0-enable
const uint8_t pll_data_array_en_output[2] =  {3, 0xFA};

const uint8_t pll_data_array_en_fanout[2] =  {187, 0xC0};//187 - 0xbb
const uint8_t pll_data_array_reset[2] =      {177, 0xAC};//177 - 0xb1

const uint8_t pll_data_array1[51] =
{
0x0F,0x00,0x4F,0x80,0x6F,0x80,0x80,0x80,0x80,0x80,0xAA,0x00,0x00,0x19,0x00,0x0A,
0xF5,0x00,0x00,0x13,0x04,0xE2,0x00,0x0A,0xF5,0x00,0x03,0x36,0x00,0x01,0x0C,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x0C,0x00,
0x00,0x00,0x00,};

void PLL_send_enable_output(void)
{
  PLL_send_data((uint8_t*)pll_data_array_en_output, 2);
}

void PLL_send_disable_output(void)
{
  PLL_send_data((uint8_t*)pll_data_array_dis_output, 2);
}

void PLL_send_fanout_enable(void)
{
  PLL_send_data((uint8_t*)pll_data_array_en_fanout, 2);
}

void PLL_send_reset(void)
{
  PLL_send_data((uint8_t*)pll_data_array_reset, 2);
}

void PLL_send_data1(void)
{
  PLL_send_data((uint8_t*)pll_data_array1, 51);
}

void configure_pll(void)
{
  PLL_send_disable_output();
  PLL_send_fanout_enable();
  PLL_send_data1();
  PLL_send_reset();
  PLL_send_enable_output();
}
