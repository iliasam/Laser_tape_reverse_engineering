//Firmware for "x-40" laser tape
//By ILIASAM
//This program captures data from ADC, calculate phase difference using Goertzel algorithm, send results to UART.
//All this done sequentially for 3 frequencies.
//UART baudrate - 256000
//Phase units - 0.1 deg
//MCU - STM32F100C8T6

#include "stm32f10x.h"
#include "config_periph.h"
#include "main.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include "pll_functions.h"
#include "analyse.h"
#include "delay_us_timer.h"
#include <stdio.h>
#include <stdlib.h>

uint16_t APD_temperature_raw = 0;//raw temperature value
float  APD_current_voltage = 80;//value in volts

AnalyseResultType result1;
AnalyseResultType result2;
AnalyseResultType result3;

int main()
{
  init_all_hardware();
  init_goertzel();

  Delay_ms(20);
  i2c_init();//must be called after vdda enabled.
  Delay_ms(200);
  
  enable_laser();
  prepare_capture();
  start_apd_voltage();
  Delay_ms(50);
  configure_pll();
  Delay_ms(100);
  
  pll_change_freq(26, 0, 1, 1250);//162.5 + 162.505
  
  while(1) 
  {
    do_single_adc_measurements();//measure temperature
    do_triple_phase_measurement();
    
    //Send results to UART
    printf("freqA_amp:%d\r\n",  result1.Amplitude);
    printf("APD temp:%d\r\n",   APD_temperature_raw);
    printf("Volt:%d\r\n",       (uint16_t)APD_current_voltage);
    printf("freqA_phase:%d\r\n", result1.Phase);
    printf("freqB_phase:%d\r\n", result2.Phase);
    printf("freqC_phase:%d\r\n", result3.Phase);
    
    auto_switch_apd_voltage((uint16_t)result1.Amplitude);//if auto switch enabled, manual switching is not working
    
    //Delay_ms(100);
  }
}

//Measure phase shifts for three frequencies
void do_triple_phase_measurement(void)
{
    //set freq1
    pll_set_frequency_1();
    dwt_delay(SWITCH_DELAY);
    result1 = do_capture();
    
    pll_set_frequency_2();
    dwt_delay(SWITCH_DELAY);
    result2 = do_capture();
    
    pll_set_frequency_3();
    dwt_delay(SWITCH_DELAY);
    result3 = do_capture();
}

void process_rx_data(uint8_t data)
{
  switch (data)
  {
      case (uint8_t)'E': //Enable laser
      {
        enable_laser();
        break;
      }
      case (uint8_t)'D': //Disable laser
      {
        disable_laser();
        break;
      }
      case (uint8_t)'H': //Set APD high voltage
      {
        set_apd_voltage(APD_HIGH_VOLTAGE);
        break;
      }
      case (uint8_t)'L': //Set APD low voltage
      {
        set_apd_voltage(APD_LOW_VOLTAGE);
        break;
      }
    
    default: break;
  }
}


int putchar(int c)
 {
   // Write a character to the USART
  USART_SendData(USART1, (u8) c);   
   
  //Loop until the end of transmission 
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)   
  {   
  }      
  return c;    
}


void Delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}






