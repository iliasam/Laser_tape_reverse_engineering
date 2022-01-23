//by ILIASAM

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config_periph.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include "stm32f0xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];
extern uint8_t capture_done;
extern uint8_t uart_disabled_flag;

uint8_t set_fine_voltage_flag = 0;
/* Private function prototypes -----------------------------------------------*/

void uart_send_data(uint8_t* data, uint16_t length);
void sent_captured_data_to_pc(void);
void process_rx_data(uint8_t data);

/* Private functions ---------------------------------------------------------*/

int main(void)
{
  delay_ms(1000);
  init_all_hardware();
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
    uint8_t rx_byte;
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){}
    rx_byte = USART_ReceiveData(USART1);
    process_rx_data(rx_byte);
  }
}

//Process data byte, received by MCU
void process_rx_data(uint8_t data)
{
  if (set_fine_voltage_flag != 0)
  {
    set_fine_voltage_flag = 0;
    set_apd_voltage((float)data);
    return;
  }
  
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
        uint32_t start_time  = HAL_GetTick();
        while(capture_done == 0)
        {
          uint32_t duration_ms = HAL_GetTick() - start_time;
          if (duration_ms > 100)//timeout
            capture_done = 1;
        }
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
      case (uint8_t)'C'://Set PLL freq4 - "FREQ LOW"
      {
        pll_set_frequency_4();//25 mhz
        break;
      }
      case (uint8_t)'1'://'one' - Set PLL freq4 - high modulation
      {
        pll_set_frequency_5();
        break;
      }
      case (uint8_t)'T':
      {
        pll_set_frequency_3();
        break;
      }
      case (uint8_t)'V'://Set fine APD voltage
      {
        set_fine_voltage_flag = 1;
        return;
        break;
      }
    default:
      break;
  }
  
  set_fine_voltage_flag = 0;
}

//send array to UART
void uart_send_data(uint8_t* data, uint16_t length)
{
  if (uart_disabled_flag)
    return;
  
  for (uint16_t i = 0; i < length; i++)
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
