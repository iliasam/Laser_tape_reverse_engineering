
#include "pll_functions.h"
#include "stm32g0xx_it.h"
#include "config_periph.h"
#include <string.h>

void pll_send_enable_output(void);

void pll_send_enable_output(void)
{
  pll_send_data((uint8_t*)"\x11\x54", 2);
  pll_send_data((uint8_t*)"\x04\x33", 2);
  pll_send_data((uint8_t*)"\x06\x33", 2);
}


void configure_pll(void)
{
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
  
  pll_set_frequency_1();
  pll_send_enable_output();
}

void pll_set_frequency_1(void)
{
  //162.0 + 161.995MHz
  pll_send_data((uint8_t*)"\x25\x02\xC7\x1C\x72", 5);
  pll_send_data((uint8_t*)"\x2d\x01", 2);
  
  pll_send_data((uint8_t*)"\x65\x02\xC7\x22\x10", 5);
  pll_send_data((uint8_t*)"\x6d\x01", 2);
}

void pll_set_frequency_2(void)
{
  //192.75 + 192.745Mhz - WRONG!!
  pll_send_data((uint8_t*)"\x25\x02\x55\xAA\x56", 5);
  pll_send_data((uint8_t*)"\x2d\x01", 2);
  
  pll_send_data((uint8_t*)"\x65\x02\x55\xAE\x4E", 5);
  pll_send_data((uint8_t*)"\x6d\x01", 2);
}


void pll_set_frequency_3(void)
{
  //193.5 + 193,495MHz
  pll_send_data((uint8_t*)"\x25\x02\x53\x59\x4D", 5);
  pll_send_data((uint8_t*)"\x2d\x01", 2);
  
  pll_send_data((uint8_t*)"\x65\x02\x53\x5D\x3E", 5);
  pll_send_data((uint8_t*)"\x6d\x01", 2);
}





