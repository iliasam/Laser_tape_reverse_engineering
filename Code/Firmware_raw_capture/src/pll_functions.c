#include "i2c_functions.h"
#include "pll_functions.h"
#include "stm32f10x_it.h"
#include "config_periph.h"
#include <string.h>

void PLL_send_enable_output(void);
void PLL_send_disable_output(void);
void PLL_send_fanout_enable(void);
void PLL_send_reset(void);
void PLL_send_data2(uint8_t start_reg, uint8_t* data, uint16_t length);
void pll_enable_divider(void);
void pll_disable_divider(void);

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

//Send data to PLL
//start_reg - address
void PLL_send_data2(uint8_t start_reg, uint8_t* data, uint16_t length)
{
  uint16_t i;
  I2C_StartTransmission(I2C_Direction_Transmitter, PLL_I2C_ADDRESS);
  I2C_WriteData(start_reg);
  for (i=0;i<length;i++)
  {
    I2C_WriteData(data[i]);
  }
  I2C_EndTransmission();
}

void configure_pll(void)
{
  PLL_send_disable_output();
  PLL_send_fanout_enable();
  PLL_send_data((uint8_t*)pll_data_array1, 51);
  PLL_send_reset();
  PLL_send_enable_output();
}

//config_reg - number, from which configuration starts
void set_pll_coeff(uint32_t a, uint32_t b, uint32_t c, uint8_t config_reg)
{
  uint32_t p1, p2, p3;
  uint8_t params[8];
  memset(params, 0, 8);
  
  //capculate "p" coefficuents
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;
  
  params[0] = (uint8_t)((p3 >> 8) & 0xFF);      //MSN_P3[15:8]
  params[1] = (uint8_t)( p3  & 0xFF);           //MSNA_P3[7:0]
  params[2] = (uint8_t)((p1 >> 16) & 0x03);     //MSN_P1[17:16]
  params[3] = (uint8_t)((p1 >> 8) & 0xFF);      //MSN_P1[15:8]
  params[4] = (uint8_t)( p1  & 0xFF);           //MSN_P1[7:0]
  params[5] = (uint8_t)((p3 >> 12) & 0xF0) + (uint8_t)((p2 >> 16) & 0x0F);//MSN_P3[19:16] + MSN_P2[19:16]
  params[6] = (uint8_t)((p2 >> 8) & 0xFF);      //MSN_P2[15:8]
  params[7] = (uint8_t)( p2  & 0xFF);           //MSN_P2[7:0]
  
  PLL_send_data2(config_reg, params, 8);
}

//pll_mult - A
//plla_coef/pllb_coef - B
//pll_div - C
//freqA = 25mhz*(MULT + A/DIV) / 4
void pll_change_freq(uint32_t pll_mult, uint32_t plla_coef, uint32_t pllb_coef, uint32_t pll_div)
{
    set_pll_coeff(pll_mult, plla_coef, pll_div, MSNA_PLL_START_REG);
    set_pll_coeff(pll_mult, pllb_coef, pll_div, MSNB_PLL_START_REG);
    PLL_send_reset();
    PLL_send_enable_output();
}

//162.5 + 162.505
void pll_set_frequency_1(void)
{
  pll_disable_divider();
  pll_change_freq(26, 0, 1, 1250);
}

//191.5 + 191.505
void pll_set_frequency_2(void)
{
  pll_disable_divider();
  pll_change_freq(30, 800, 801, 1250);
}

//193.5 + 193.505
void pll_set_frequency_3(void)
{
  pll_disable_divider();
  pll_change_freq(30, 1200, 1201, 1250);
}
//25 mhz
void pll_set_frequency_4(void)
{
  pll_enable_divider();
  pll_change_freq(16, 0, 4, 1250);
}

void pll_enable_divider(void)
{
  //(DIV = 1) - 16 - div 16
  //(DIV = 2) - 32 - div 32
  const uint8_t pll_data_array_div_enableA[2] =  {44, 16};//ms0
  const uint8_t pll_data_array_div_enableB[2] =  {60, 16};//ms2
  PLL_send_data((uint8_t*)pll_data_array_div_enableA, 2);
  PLL_send_data((uint8_t*)pll_data_array_div_enableB, 2);
}

void pll_disable_divider(void)
{
  const uint8_t pll_data_array_div_enableA[2] =  {44, 12};//ms0 - Divide by 4 enabled.
  const uint8_t pll_data_array_div_enableB[2] =  {60, 12};//ms2- Divide by 4 enabled.
  PLL_send_data((uint8_t*)pll_data_array_div_enableA, 2);
  PLL_send_data((uint8_t*)pll_data_array_div_enableB, 2);
}