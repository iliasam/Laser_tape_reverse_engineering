//3 frequencies
//This programm calculate distance to object in mm
//Plain data processing
//UART baudrate - 256000
//MCU - STM32F100C8T6
//By ILIASAM

#include "stm32f10x.h"
#include "config_periph.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include "i2c_functions.h"
#include "analyse.h"
#include "delay_us_timer.h"
#include "measure_functions.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"

extern volatile uint8_t capture_done;

uint16_t APD_temperature_raw = 0;//raw temperature value
uint8_t  APD_current_voltage = 82;//value in volts

extern AnalyseResultType result1;

uint8_t measure_enabled = 1;//auto distance measurement enabled flag
uint8_t calibration_needed = 0;//1- calibration needed flag

//for testing purposes
volatile uint32_t delta_time = 0;

int main()
{
  init_all_hardware();
  init_goertzel();
  read_calib_data_from_flash();

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
  
  printf("Start\r\n");
  
  while(1) 
  {
    static uint32_t old_dwt_value = 0;//for testing purposes
    
    if (measure_enabled == 1)
    {
      uint32_t cur_dwt_value = get_dwt_value();
      delta_time = cur_dwt_value - old_dwt_value;
      old_dwt_value = cur_dwt_value;
      
      do_single_adc_measurements();//measure temperature
      do_triple_phase_measurement();
      do_distance_calculation();
      auto_switch_apd_voltage(result1.Amplitude);//if auto switch enabled, manual switching is not working
    }
    
    if (calibration_needed == 1)
    {
      do_phase_calibration();
      calibration_needed = 0;
    }    
  }
}

void process_rx_data(uint8_t data)
{
  switch (data)
  {
      case (uint8_t)'E': //Enable laser and measurement process
      {
        enable_laser();
        measure_enabled = 1;
        break;
      }
      case (uint8_t)'D': //Disable laser and measurement process
      {
        disable_laser();
        measure_enabled = 0;
        break;
      }
      case (uint8_t)'H':
      {
        switch_apd_voltage(95); //Set APD high voltage
        break;
      }
      case (uint8_t)'L':
      {
        switch_apd_voltage(82); //Set APD low voltage
        break;
      }
      case (uint8_t)'C'://Start calibration process
      {
        calibration_needed = 1;
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