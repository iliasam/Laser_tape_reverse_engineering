#include "stm32g0xx.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "config_periph.h"

#include "stm32g0xx_it.h"


/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
float  APD_current_voltage = 0.0f;//value in volts

extern volatile uint32_t ms_uptime;


/* Private function prototypes -----------------------------------------------*/
void init_gpio(void);
void init_uart1(void);
void init_volt_pwm_timer(void);
void init_i2c(void);


/* Private functions ---------------------------------------------------------*/

void init_all_hardware(void)
{
  init_gpio();
  init_volt_pwm_timer();
  init_uart1();
  delay_ms(20);
}

void init_gpio(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  
  LL_GPIO_SetPinMode(BEEP_PORT, BEEP_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(BEEP_PORT, BEEP_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_ResetOutputPin(BEEP_PORT, BEEP_PIN);
  
  LL_GPIO_SetPinMode(MAIN_POWER_PORT, MAIN_POWER_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(MAIN_POWER_PORT, MAIN_POWER_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_ResetOutputPin(MAIN_POWER_PORT, MAIN_POWER_PIN); //enable power
  
  LL_GPIO_SetPinMode(ANALOG_POWER_PORT, ANALOG_POWER_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(ANALOG_POWER_PORT, ANALOG_POWER_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetOutputPin(ANALOG_POWER_PORT, ANALOG_POWER_PIN); //enable power 
  
  LL_GPIO_SetPinMode(GPIOA, ADC_REF_PIN, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(GPIOA, ADC_SIGNAL_PIN, LL_GPIO_MODE_ANALOG);
  
  LL_GPIO_SetPinMode(VOLT_PWM_TIMER_LASER_CTRL_PORT, 
    VOLT_PWM_TIMER_LASER_CTRL_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(VOLT_PWM_TIMER_LASER_CTRL_PORT, 
    VOLT_PWM_TIMER_LASER_CTRL_PIN, LL_GPIO_MODE_ALTERNATE);
    
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = VOLT_PWM_TIMER_APD_CTRL_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = VOLT_PWM_TIMER_APD_CTRL_AF;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = VOLT_PWM_TIMER_LASER_CTRL_PIN;
  GPIO_InitStruct.Alternate = VOLT_PWM_TIMER_LASER_CTRL_AF;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void init_uart1(void)
{

  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  GPIO_InitStruct.Pin = UART_TX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = UART_GPIO_AF;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = UART_RX_PIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_USART_DeInit(USART1);
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = UART_BAURDATE;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || 
    (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
}



//new_voltage - target APD voltage in volts
void set_apd_voltage(float new_voltage)
{
  if ((new_voltage > 115.0f) || (new_voltage < 70.0f))
    return;
  
  float dac_voltage = 0.0f;
  float tmp1 = DCDC_VREF * (1 + (DCDC_R_UP / DCDC_R_DOWN));
  
  dac_voltage = DCDC_VREF - (new_voltage * APD_CORR_COEF - tmp1) * (DCDC_R_DAC / DCDC_R_UP);
  
  float pwm_value = dac_voltage * VOLT_PWM_TIMER_PERIOD / AREF_VOLTAGE;
  LL_TIM_OC_SetCompareCH2(VOLT_PWM_TIMER_NAME, (uint16_t)pwm_value);
  APD_current_voltage = new_voltage;
}

void enable_laser(void)
{
  LL_TIM_OC_SetCompareCH3(VOLT_PWM_TIMER_NAME, VOLT_PWM_TIMER_LASER_DUTY);
}

void disable_laser(void)
{
  LL_TIM_OC_SetCompareCH3(VOLT_PWM_TIMER_NAME, 0);
}


void init_volt_pwm_timer(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  VOLT_PWM_TIMER_RCC_CMD(VOLT_PWM_TIMER_RCC);
  LL_TIM_DeInit(VOLT_PWM_TIMER_NAME);
  
  LL_TIM_StructInit(&TIM_InitStruct);
  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);

  TIM_InitStruct.Prescaler = VOLT_PWM_TIMER_PRESCALER;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = VOLT_PWM_TIMER_PERIOD - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(VOLT_PWM_TIMER_NAME, &TIM_InitStruct);
  
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 10;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  
  LL_TIM_OC_DisableFast(VOLT_PWM_TIMER_NAME, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisableFast(VOLT_PWM_TIMER_NAME, LL_TIM_CHANNEL_CH3);
  
  LL_TIM_EnableARRPreload(VOLT_PWM_TIMER_NAME);
  LL_TIM_OC_EnablePreload(VOLT_PWM_TIMER_NAME, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(VOLT_PWM_TIMER_NAME, LL_TIM_CHANNEL_CH3);
  
  LL_TIM_OC_Init(VOLT_PWM_TIMER_NAME, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_Init(VOLT_PWM_TIMER_NAME, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  
  LL_TIM_SetTriggerOutput(VOLT_PWM_TIMER_NAME, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(VOLT_PWM_TIMER_NAME);
  
  LL_TIM_CC_EnableChannel(VOLT_PWM_TIMER_NAME, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableCounter(VOLT_PWM_TIMER_NAME);
}

void delay_ms(uint32_t ms)
{
  uint32_t start_ms = ms_uptime;
  while ((ms_uptime - start_ms) < ms) {}
}

