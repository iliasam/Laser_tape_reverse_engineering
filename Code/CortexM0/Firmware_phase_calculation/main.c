//Firmware for "x-40" laser tape
//By ILIASAM
//This program captures data from ADC, calculate phase difference using Goertzel algorithm, send results to UART.
//All this done sequentially for 3 frequencies.
//UART baudrate - 256000
//Phase units - 0.1 deg

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config_periph.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include "analyse.h"
#include "stdio.h"
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t APD_temperature_raw = 0;//raw temperature value
float  APD_temperature = 0.0;//temperature value in deg
float  APD_current_voltage = 80;//value in volts

extern uint8_t uart_disabled_flag;

AnalyseResultType result1;
AnalyseResultType result2;
AnalyseResultType result3;

/* Private function prototypes -----------------------------------------------*/
void do_triple_phase_measurement(void);
void process_rx_data(uint8_t data);

/* Private functions ---------------------------------------------------------*/

int main(void)
{
  delay_ms(1000);
  init_all_hardware();
  init_goertzel();
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
  start_adc_capture();
  
  while (1)
  {
    capture_do_single_adc_measurements();//measure temperature
    do_triple_phase_measurement();
    

    //Send results to UART
    printf("freqA_amp:%d\r\n",  result1.Amplitude);
    printf("APD temp:%d\r\n",   APD_temperature_raw);
    printf("Volt:%d\r\n",       (uint16_t)APD_current_voltage);
    printf("freqA_phase:%d\r\n", result1.Phase);
    printf("freqB_phase:%d\r\n", result2.Phase);
    printf("freqC_phase:%d\r\n", result3.Phase);
    
    //if auto switch enabled, manual switching is not working
    auto_switch_apd_voltage((uint16_t)result1.Amplitude);
    
    //Delay_ms(100);
  }
}

//Measure phase shifts for three frequencies
void do_triple_phase_measurement(void)
{
    //init for main signal capture
    prepare_capture();
    
    //set freq1
    pll_set_frequency_1();
    //dwt_delay(SWITCH_DELAY);
    delay_ms(1);
    result1 = do_capture();
    
    pll_set_frequency_2();
    //dwt_delay(SWITCH_DELAY);
    delay_ms(1);
    result2 = do_capture();
    
    pll_set_frequency_3();
    //dwt_delay(SWITCH_DELAY);
    delay_ms(1);
    result3 = do_capture();
}

//Process data byte, received by MCU
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
