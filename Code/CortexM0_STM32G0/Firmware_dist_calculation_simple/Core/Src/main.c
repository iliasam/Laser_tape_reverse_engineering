//Firmware for "x-40" laser tape
//By ILIASAM
//This program captures data from ADC, calculate phase difference using Goertzel algorithm, send results to UART.
//All this done sequentially for 3 frequencies.
//UART baudrate - 256000
//Phase units - 0.1 deg

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "config_periph.h"
#include "i2c_functions.h"
#include "pll_functions.h"
#include "capture_configure.h"
#include "delay_us_timer.h"
#include "measure_functions.h"
#include "analyse.h"
#include "stdio.h"
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

struct __FILE
{
  int dummyVar; //Just for the sake of redefining __FILE, we won't we using it anyways ;)
};

FILE __stdout;
FILE __stdin;

uint16_t APD_temperature_raw = 0;//raw temperature value
float  APD_temperature_deg = 30.0f;//temperature value in deg
float  APD_current_voltage = 80.0f;//value in volts

uint8_t measure_enabled = 1;//auto distance measurement enabled flag
uint8_t calibration_needed = 0;//1- calibration needed flag

extern AnalyseResultType result1;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void process_rx_data(uint8_t data);
void send_captured_data_to_pc(void);
void uart_send_data(uint8_t* data, uint16_t length);
void do_triple_phase_measurement(void);

/* Private user code ---------------------------------------------------------*/

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure the system clock */
  SystemClock_Config();

  //enable systick first!
  LL_SYSTICK_EnableIT();
  init_all_hardware();
  dwt_init();
  init_i2c();
  read_calib_data_from_flash();
  delay_ms(200);
  enable_laser();
  set_apd_voltage(APD_DEFAULT_VOLTAGE);
  init_goertzel();
  delay_ms(50);
  configure_pll();
  prepare_capture();
  delay_ms(100);

  
  printf("Start\r\n");

  /* Infinite loop */
  while (1)
  {
    if (measure_enabled == 1)
    {
      do_triple_phase_measurement();
      capture_do_single_adc_measurements();//measure temperature
      do_distance_calculation();
      //if auto switch enabled, manual switching is not working
      auto_switch_apd_voltage(result1.Amplitude);
    }
    
    if (calibration_needed == 1)
    {
      do_phase_calibration();
      calibration_needed = 0;
    }
    
    
    if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART1))
    {
      uint8_t rx_byte;
      rx_byte = LL_USART_ReceiveData8(USART1);
      process_rx_data(rx_byte);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
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
      case (uint8_t)'C'://Start calibration process
      {
        calibration_needed = 1;
        break;
      }
    
    default: break;
  }
}

//send array to UART
void uart_send_data(uint8_t* data, uint16_t length)
{
  for (uint16_t i = 0; i < length; i++)
  {
    while(LL_USART_IsActiveFlag_TXE_TXFNF(USART1) == 0) {} //while not empty
    LL_USART_TransmitData8(USART1, (uint8_t)data[i]);  
  }
}

int fputc(int c, FILE * stream)
{
  while(LL_USART_IsActiveFlag_TXE_TXFNF(USART1) == 0) {} //while not empty
  LL_USART_TransmitData8(USART1, (uint8_t)c);  
  return c;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
