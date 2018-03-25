//Firmware for "x-40" laser tape
//By ILIASAM
//This program configures PLL and other tape peripherals
//After receiving "M" symbol, program start capturing data from two ADC channels
//After capture is done, program send data to PC
//UART baudrate - 256000
//MCU - STM32F100C8T6 
//Supported 512A / 701A board revisions - see "main.h"


#include "stm32f10x.h"
#include "config_periph.h"
#include "main.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include <stdio.h>
#include <stdlib.h>

extern uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];
extern uint8_t capture_done;

void process_rx_data(uint8_t data);
void sent_captured_data_to_pc(void);
void uart_send_data(uint8_t* data, uint16_t length);

int main()
{
  uint8_t rx_byte  = 0;
  RCC_ClocksTypeDef RCC_Clocks;
  
  init_sys_clock();
  RCC_GetClocksFreq (&RCC_Clocks);
  if (SysTick_Config (RCC_Clocks.SYSCLK_Frequency/1000)) { /* Setup SysTick for 1 msec interrupts */;while (1);}
  
  init_dac();
  init_gpio();
  init_uart1();
  Delay_ms(20);
  i2c_init();//must be called after vdda enabled.
  Delay_ms(200);
  
  enable_laser();
  
  start_apd_voltage();
  Delay_ms(50);
  configure_pll();
  Delay_ms(100);
  
  pll_set_frequency_1();
  
  prepare_capture();
  
  start_adc_capture();
  Delay_ms(2000);
  
  while(1) 
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){}
    rx_byte = USART_ReceiveData (USART1);
    process_rx_data(rx_byte);
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
        break;
      }
      case (uint8_t)'D': //Disable laser
      {
        disable_laser();
        break;
      }
      case (uint8_t)'M': //Start raw data capture
      {
        start_adc_capture();
        while(capture_done == 0){}
        sent_captured_data_to_pc();
        break;
      }
      case (uint8_t)'S': //Set APD super high voltage
      {
        set_apd_voltage(APD_SHIGH_VOLTAGE);
        break;
      }
      case (uint8_t)'H': //Set APD high voltage
      {
        set_apd_voltage(APD_HIGH_VOLTAGE);
        break;
      }
      case (uint8_t)'L'://Set APD low voltage
      {
        set_apd_voltage(APD_LOW_VOLTAGE);
        break;
      }
      case (uint8_t)'A'://Set PLL freq1
      {
        pll_set_frequency_1();
        break;
      }
      case (uint8_t)'B'://Set PLL freq2
      {
        pll_set_frequency_2();
        break;
      }
      case (uint8_t)'C'://Set PLL freq4 - low
      {
        pll_set_frequency_4();
        break;
      }
    
    default: break;
  }
}


void Delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}

//send array to UART
void uart_send_data(uint8_t* data, uint16_t length)
{
  uint16_t i;
  
  for (i=0;i<length;i++)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {} //while not empty
    USART_SendData(USART1, (uint8_t)data[i]);  
  }
}

void sent_captured_data_to_pc(void)
{
  uart_send_data((uint8_t*)"ABCD",4);//header
  uart_send_data((uint8_t*)&adc_capture_buffer[0], (ADC_CAPURE_BUF_LENGTH*2));//2 * uint16_t points
}






