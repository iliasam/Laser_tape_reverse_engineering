//3 frequencies
//This program calculate distance to object in mm
//Plain data processing
//UART baudrate - 256000
//MCU - STM32F030F4P6?
//By ILIASAM

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config_periph.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include "analyse.h"
#include "measure_functions.h"
#include "delay_us_timer.h"
#include "stdio.h"
#include "uart_handler.h"
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t APD_temperature_raw = 0;//raw temperature value
float  APD_temperature = 0.0;//temperature value in deg
float  APD_current_voltage = 80;//value in volts

uint8_t measure_enabled = 1;//auto distance measurement enabled flag
uint8_t calibration_needed = 0;//1- calibration needed flag

extern uint8_t uart_disabled_flag;
extern AnalyseResultType result1;

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

int main(void)
{
  delay_ms(1000);
  init_all_hardware();
  init_uart1();
  dwt_init();
  init_goertzel();
  read_calib_data_from_flash();
  init_i2c();
  delay_ms(200);
  
  enable_laser();
  set_apd_voltage(APD_DEFAULT_VOLTAGE);
  delay_ms(50);
  configure_pll();
  delay_ms(100);
  
  pll_set_frequency_1();
  delay_ms(100);
  
  prepare_capture();
  
  printf("Start\r\n");
  
  while (1)
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


//Process data byte, received by MCU
void process_rx_data(uint8_t data)
{
  switch (data)
  {
      case (uint8_t)'E': //Enable laser
      {
        enable_laser();
        measure_enabled = 1;
        break;
      }
      case (uint8_t)'D': //Disable laser
      {
        disable_laser();
        measure_enabled = 0;
        break;
      }
      /*
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
      */
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
  if (uart_disabled_flag)
    return c;
   
  // Write a character to the USART
  USART_SendData(UART_NAME, (uint8_t)c);
   
  //Loop until the end of transmission 
  while(USART_GetFlagStatus(UART_NAME, USART_FLAG_TXE) == RESET)
  {   
  }
 
  return c;    
}
