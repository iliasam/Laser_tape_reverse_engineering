//3 frequencies
//This programm calculate distance to object in mm
//Simultaneous data capture and data processing
//UART baudrate - 256000
//MCU - STM32F100C8T6
//By ILIASAM

#include "stm32f10x.h"
#include "config_periph.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include "i2c_functions.h"
#include "measure_functions.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"

uint16_t APD_temperature_raw = 1000;//raw temperature value
uint8_t  APD_current_voltage = 82;//value in volts

uint8_t measure_enabled = 1;//auto distance measurement enabled flag
uint8_t calibration_needed = 0;


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
    
    if (measure_enabled == 1)
    {
        auto_handle_capture();
        auto_handle_data_processing();
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






