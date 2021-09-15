//Open Simple Lidar v4
//Testing code - blinking LED when encoder event occurs
//UART TX pin is used as GPIO pin
//by ILIASAM

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config_periph.h"
#include "pll_functions.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
  delay_ms(1000);
  init_all_hardware();
  delay_ms(200);
  
  enable_laser();
  set_apd_voltage(APD_DEFAULT_VOLTAGE);
  delay_ms(50);
  configure_pll();
  delay_ms(100);
  
  pll_set_frequency_1();
  
  //prepare_capture();
  
  //start_adc_capture();
  delay_ms(500);
  
  while (1)
  {

  }
}
