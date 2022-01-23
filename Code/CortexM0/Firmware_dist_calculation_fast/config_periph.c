#include "stm32f0xx.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "config_periph.h"
#include "stm32f0xx_i2c.h"
#include "capture_configure.h"


/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern float  APD_current_voltage;//value in volts
extern uint16_t APD_temperature_raw;
extern float APD_temperature_deg;
extern uint8_t apd_voltage_decrease;

float test_apd_voltage = 90.0f;
uint8_t apd_voltage_reduced_flag = 0;
uint16_t corrected_amplitude = 0;

/* Private function prototypes -----------------------------------------------*/
void init_sys_clock(void);
void init_gpio(void);
void init_volt_pwm_timer(void);
void init_i2c(void);


/* Private functions ---------------------------------------------------------*/

void init_all_hardware(void)
{
  init_sys_clock();
  SystemCoreClockUpdate();
  if (SysTick_Config(SystemCoreClock / 1000)) { /* Setup SysTick for 1 msec interrupts */;while (1);}
  
  init_gpio();
  init_volt_pwm_timer();
  delay_ms(20);
}

void init_gpio(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
   
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | 
                        RCC_AHBPeriph_GPIOF, ENABLE );
  
  GPIO_InitStructure.GPIO_Pin   = BEEP_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(BEEP_PORT, BEEP_PIN);

  GPIO_InitStructure.GPIO_Pin   = MAIN_POWER_PIN;
  GPIO_Init(MAIN_POWER_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(MAIN_POWER_PORT, MAIN_POWER_PIN);//enable power
  
  GPIO_InitStructure.GPIO_Pin   = ANALOG_POWER_PIN;
  GPIO_Init(ANALOG_POWER_PORT, &GPIO_InitStructure);
  GPIO_SetBits(ANALOG_POWER_PORT, ANALOG_POWER_PIN);//enable power

  GPIO_InitStructure.GPIO_Pin   = (ADC_REF_PIN | ADC_SIGNAL_PIN);//adc pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = ADC_TEMP_PIN;//adc pin - temperature
  GPIO_Init(ADC_TEMP_PORT, &GPIO_InitStructure);
  
  //Timer for controlling APD voltage and laser power
  GPIO_InitStructure.GPIO_Pin = VOLT_PWM_TIMER_APD_CTRL_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(VOLT_PWM_TIMER_APD_CTRL_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(
    VOLT_PWM_TIMER_APD_CTRL_PORT, VOLT_PWM_TIMER_APD_CTRL_SRC, 
    VOLT_PWM_TIMER_APD_CTRL_AF);//timer2
  
  GPIO_InitStructure.GPIO_Pin = VOLT_PWM_TIMER_LASER_CTRL_PIN;
  GPIO_Init(VOLT_PWM_TIMER_LASER_CTRL_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(
    VOLT_PWM_TIMER_LASER_CTRL_PORT, VOLT_PWM_TIMER_LASER_CTRL_SRC, 
    VOLT_PWM_TIMER_LASER_CTRL_AF);//timer2
}



//new_voltage - target APD voltage in volts
void set_apd_voltage(float new_voltage)
{   
  if (new_voltage > APD_MAX_VOLTAGE_V)
      new_voltage = APD_MAX_VOLTAGE_V;
  
  if (new_voltage < 70.0f)
    new_voltage = 70.0f;
  
  float dac_voltage = 0.0f;
  float tmp1 = DCDC_VREF * (1 + (DCDC_R_UP / DCDC_R_DOWN));
  
  dac_voltage = DCDC_VREF - (new_voltage * APD_CORR_COEF - tmp1) * (DCDC_R_DAC / DCDC_R_UP);
  
  float pwm_value = dac_voltage * VOLT_PWM_TIMER_PERIOD / AREF_VOLTAGE;
  TIM_SetCompare2(VOLT_PWM_TIMER_NAME, (uint16_t)pwm_value);
  APD_current_voltage = new_voltage;
}

void enable_laser(void)
{
  TIM_SetCompare3(VOLT_PWM_TIMER_NAME, VOLT_PWM_TIMER_LASER_DUTY);
}

void disable_laser(void)
{
  TIM_SetCompare3(VOLT_PWM_TIMER_NAME, 0);
}


void init_sys_clock(void)
{
  RCC_DeInit(); /* RCC system reset(for debug purpose) */

  FLASH_PrefetchBufferCmd(ENABLE);  // Enable Prefetch Buffer
  FLASH_SetLatency(FLASH_Latency_1);// Flash 1 wait state
  
  RCC_HCLKConfig(RCC_SYSCLK_Div1);// HCLK = SYSCLK
  RCC_PCLKConfig(RCC_HCLK_Div1);  // APB  = HCLK
  
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);// PLLCLK = 8MHz / 2 * 12 = 48 MHz
  RCC_PLLCmd(ENABLE);/* Enable PLL */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};/* Wait till PLL is ready */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);/* Select PLL as system clock source */
  while (RCC_GetSYSCLKSource() != 0x08){}/* Wait till PLL is used as system clock source */
}


void init_volt_pwm_timer(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  VOLT_PWM_TIMER_RCC_CMD(VOLT_PWM_TIMER_RCC, ENABLE);
  
  TIM_DeInit(VOLT_PWM_TIMER_NAME);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Prescaler = VOLT_PWM_TIMER_PRESCALER; 
  TIM_TimeBaseStructure.TIM_Period = VOLT_PWM_TIMER_PERIOD - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(VOLT_PWM_TIMER_NAME, &TIM_TimeBaseStructure);
  
  // channel2 - MCU_APD_CTRL 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = 10; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;         
  TIM_OC2Init(VOLT_PWM_TIMER_NAME, &TIM_OCInitStructure);
  
  // channel3 - LASER_POWER_PWM  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = VOLT_PWM_TIMER_LASER_DUTY; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;         
  TIM_OC3Init(VOLT_PWM_TIMER_NAME, &TIM_OCInitStructure);
  
  TIM_OC2PreloadConfig(VOLT_PWM_TIMER_NAME, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(VOLT_PWM_TIMER_NAME, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(VOLT_PWM_TIMER_NAME, ENABLE);
  
  TIM_CtrlPWMOutputs(VOLT_PWM_TIMER_NAME, ENABLE);
  TIM_Cmd(VOLT_PWM_TIMER_NAME, ENABLE);
}

void delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}

//Calculate temperature in degrees from raw ADC value
void calculate_real_temperature(uint16_t raw_value)
{
  float result = 66.843f;
  result+= -0.029572f * (float)raw_value;
  result+= (2.122e-6f) * (float)raw_value * (float)raw_value;
  APD_temperature_deg = result;
}

void auto_switch_apd_voltage(uint16_t current_amplitude)
{
  //Kind of AGC
  if (apd_voltage_reduced_flag)
  {
    corrected_amplitude = current_amplitude * 2;
    if (corrected_amplitude < SIGNAL_AMPL_INCREASE_LEVEL)
    {
      apd_voltage_reduced_flag = 0;
    }
  }
  else
  {
    corrected_amplitude = current_amplitude;
    if (corrected_amplitude > SIGNAL_AMPL_REDUCE_LEVEL)
    {
      apd_voltage_reduced_flag = 1;
    }
  }
  
  //APD voltage is depending only from a temperature ?
  //"APD_temperature_deg" value in deg
  float voltage_to_set_v = 0.4866667f * APD_temperature_deg + 90.0f; //TODO
  if (apd_voltage_reduced_flag)
    voltage_to_set_v -= apd_voltage_decrease;
  set_apd_voltage(voltage_to_set_v);
  //set_apd_voltage(test_apd_voltage);
  
}


