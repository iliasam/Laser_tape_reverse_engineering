
#include "pll_functions.h"
#include "stm32g0xx_it.h"
#include "config_periph.h"
#include <string.h>

void pll_send_enable_output(void);
void pll_send_disable_output(void);
void pll_send_fanout_enable(void);
void pll_send_reset(void);
void pll_enable_divider(void);
void pll_disable_divider(void);

const uint8_t pll_data_array_dis_output[2] = {3, 0xff};//0-enable
const uint8_t pll_data_array_en_output[2] =  {3, 0xFA};


const uint8_t pll_data_array_reset[2] =      {177, 0xAC};//177 - 0xb1

const uint8_t pll_data_array1[51] =
{
0x0F,0x00,0x4F,0x80,0x6F,0x80,0x80,0x80,0x80,0x80,0xAA,0xFF,0x00,0x19,0x00,0x0A,
0xF5,0x00,0x00,0x13,0x04,0xE2,0x00,0x0A,0xF5,0x00,0x03,0x36,0x00,0x01,0x0C,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x0C,0x00,
0x00,0x00,0x00,};

void pll_send_enable_output(void)
{
  pll_send_data((uint8_t*)"\x11\x54", 2);
  pll_send_data((uint8_t*)"\x04\x33", 2);
  pll_send_data((uint8_t*)"\x06\x33", 2);
}

void pll_send_disable_output(void)
{
  pll_send_data((uint8_t*)pll_data_array_dis_output, 2);
}

void pll_send_fanout_enable(void)
{

}

void pll_send_reset(void)
{
  pll_send_data((uint8_t*)pll_data_array_reset, 2);
}



void configure_pll(void)
{
  //pll_send_data((uint8_t*)pll_data_array1, 51);
  
  pll_send_data((uint8_t*)"\xB6", 1);
  pll_send_data((uint8_t*)"\x01", 1);
  pll_send_data((uint8_t*)"\x02\x80", 2);
  pll_send_data((uint8_t*)"\x04\x30", 2);
  pll_send_data((uint8_t*)"\x05\x32", 2);
  pll_send_data((uint8_t*)"\x06\x30", 2);
  pll_send_data((uint8_t*)"\x07\x0B", 2);
  pll_send_data((uint8_t*)"\x08\xF4", 2);
  pll_send_data((uint8_t*)"\x10\x80", 2);
  pll_send_data((uint8_t*)"\x11\x04", 2);
  
  pll_send_data((uint8_t*)"\x20\x80", 2);
  pll_send_data((uint8_t*)"\x21\x02", 2);
  pll_send_data((uint8_t*)"\x22\x42", 2);
  pll_send_data((uint8_t*)"\x30\x00", 2);
  pll_send_data((uint8_t*)"\x31\x00", 2);
  pll_send_data((uint8_t*)"\x32\x02", 2);
  pll_send_data((uint8_t*)"\x33\x01", 2);
  
  pll_send_data((uint8_t*)"\x60\x80", 2);
  pll_send_data((uint8_t*)"\x61\x02", 2);
  pll_send_data((uint8_t*)"\x62\x42", 2);
  pll_send_data((uint8_t*)"\x70\x00", 2);
  pll_send_data((uint8_t*)"\x71\x00", 2);
  pll_send_data((uint8_t*)"\x72\x02", 2);
  pll_send_data((uint8_t*)"\x73\x01", 2);
  
  delay_ms(100);
  
  pll_send_data((uint8_t*)"\x25\x02\x61\x86\x18", 5);
  pll_send_data((uint8_t*)"\x2d\x01", 2);
  
  pll_send_data((uint8_t*)"\x65\x02\x61\x8a\x39", 5);
  pll_send_data((uint8_t*)"\x6d\x01", 2);
  
  pll_send_enable_output();
}

//config_reg - number, from which configuration starts
void set_pll_coeff(uint32_t a, uint32_t b, uint32_t c, uint8_t config_reg)
{
  uint32_t p1, p2, p3;
  uint8_t params[8];
  memset(params, 0, 8);
  
  //calculate "p" coefficuents
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
  
  pll_send_data2(config_reg, params, 8);
}

//pll_mult - common multiplication
//plla_coef/pllb_coef - outputA/outputB coefficients
//pll_div - common divider
//freqA = 25mhz*(pll_mult + plla_coef/pll_div) / 4
//freqB = 25mhz*(pll_mult + pllb_coef/pll_div) / 4
//Divider to 4 can be switched
void pll_change_freq(uint32_t pll_mult, uint32_t plla_coef, uint32_t pllb_coef, uint32_t pll_div)
{
    set_pll_coeff(pll_mult, plla_coef, pll_div, MSNA_PLL_START_REG);
    set_pll_coeff(pll_mult, pllb_coef, pll_div, MSNB_PLL_START_REG);
    pll_send_reset();
    pll_send_enable_output();
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


void pll_set_frequency_3(void)
{
  pll_disable_divider();
  //diff is 4.8 kHz
  set_pll_coeff(25, 14323, 15625, MSNA_PLL_START_REG); //161.9792 MHz
  set_pll_coeff(25, 28622, 31250, MSNB_PLL_START_REG); //161.9744 MHz
  pll_send_reset();
  pll_send_enable_output();
}

//25 mhz
void pll_set_frequency_4(void)
{
  pll_enable_divider();
  pll_change_freq(16, 0, 4, 1250);
}


void pll_set_frequency_5(void)
{
  //10 khz
  pll_disable_divider();
  pll_change_freq(30, 1200, 1202, 1250);
}

// Set divider to 16
void pll_enable_divider(void)
{
  //(DIV = 1) - 16 - div 16
  //(DIV = 2) - 32 - div 32
  const uint8_t pll_data_array_div_enableA[2] =  {44, 16};//ms0
  const uint8_t pll_data_array_div_enableB[2] =  {60, 16};//ms2
  pll_send_data((uint8_t*)pll_data_array_div_enableA, 2);
  pll_send_data((uint8_t*)pll_data_array_div_enableB, 2);
}

// Set divider to 4
void pll_disable_divider(void)
{
  const uint8_t pll_data_array_div_enableA[2] =  {44, 12};//ms0 - Divide by 4 enabled.
  const uint8_t pll_data_array_div_enableB[2] =  {60, 12};//ms2- Divide by 4 enabled.
  pll_send_data((uint8_t*)pll_data_array_div_enableA, 2);
  pll_send_data((uint8_t*)pll_data_array_div_enableB, 2);
}

