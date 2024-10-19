/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "config_periph.h"
#include "i2c_functions.h"
#include "pll_functions.h"
#include "capture_configure.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t set_fine_voltage_flag = 0;
extern volatile uint32_t ms_uptime;
extern volatile uint8_t capture_done;
extern uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


void process_rx_data(uint8_t data);
void send_captured_data_to_pc(void);
void uart_send_data(uint8_t* data, uint16_t length);


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
  init_i2c();
  delay_ms(200);
  enable_laser();
  set_apd_voltage(APD_DEFAULT_VOLTAGE);
  delay_ms(50);
  configure_pll();
  prepare_capture();
  delay_ms(100);

  delay_ms(100);

  /* Infinite loop */
  while (1)
  {
    uint8_t rx_byte;
    while (LL_USART_IsActiveFlag_RXNE_RXFNE(USART1) == 0){}
    rx_byte = LL_USART_ReceiveData8(USART1);
    process_rx_data(rx_byte);
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
        uint32_t start_time  = ms_uptime;
        while(capture_done == 0)
        {
          uint32_t duration_ms = ms_uptime - start_time;
          if (duration_ms > 100)//timeout
            capture_done = 1;
        }
        send_captured_data_to_pc();
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
      }
    default:
      break;
  }
  
  set_fine_voltage_flag = 0;
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

void send_captured_data_to_pc(void)
{
  uart_send_data((uint8_t*)"ABCD",4);//header
  uart_send_data((uint8_t*)adc_capture_buffer, (ADC_CAPURE_BUF_LENGTH * 2));//2 * uint16_t points
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
