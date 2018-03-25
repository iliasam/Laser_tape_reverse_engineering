#include "stm32f10x.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "config_periph.h"
#include "capture_configure.h"
#include "stm32f10x_i2c.h"
#include "delay_us_timer.h"

extern uint16_t APD_temperature_raw;
extern float  APD_current_voltage;//value in volts

void init_all_hardware(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  init_sys_clock();
  RCC_GetClocksFreq (&RCC_Clocks);
  if (SysTick_Config (RCC_Clocks.SYSCLK_Frequency/1000)) { /* Setup SysTick for 1 msec interrupts */;while (1);}
  
  dwt_init();
  init_dac();
  init_gpio();
  init_uart1();
}

void init_gpio(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin   = BEEP_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
  
  GPIO_SetBits(LASER_PORT, LASER_POWER_PIN);
  GPIO_InitStructure.GPIO_Pin   = LASER_POWER_PIN;
  GPIO_Init(LASER_PORT, &GPIO_InitStructure);
  GPIO_SetBits(LASER_PORT, LASER_POWER_PIN);
  
  GPIO_InitStructure.GPIO_Pin   = MAIN_POWER_PIN;
  GPIO_Init(MAIN_POWER_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(MAIN_POWER_PORT, MAIN_POWER_PIN);//enable power
  
  GPIO_InitStructure.GPIO_Pin   = ANALOG_POWER_PIN;
  GPIO_Init(ANALOG_POWER_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(ANALOG_POWER_PORT, ANALOG_POWER_PIN);//enable power

  GPIO_InitStructure.GPIO_Pin   = (ADC_REF_PIN | ADC_SIGNAL_LOW_PIN | ADC_SIGNAL_HIGH_PIN);//adc pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void init_dac(void)
{
  DAC_InitTypeDef  DAC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;//dac1 - dac2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  DAC_StructInit(&DAC_InitStructure);
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  //laser voltage
  DAC_Cmd(DAC_Channel_1, ENABLE);
  DAC_SetChannel1Data(DAC_Align_12b_R, LASER_DAC1_VALUE);
  
  //APD voltage
  DAC_Cmd(DAC_Channel_2, ENABLE);
  DAC_SetChannel2Data(DAC_Align_12b_R, APD_DAC2_VALUE1);
}

void enable_laser(void)
{
  LASER_PORT->ODR&= ~LASER_POWER_PIN;
}

void disable_laser(void)
{
  LASER_PORT->ODR|= LASER_POWER_PIN;
}

//create power sequence for APD
void start_apd_voltage(void)
{
  //APD voltage
  DAC_SetChannel2Data(DAC_Align_12b_R, APD_DAC2_VALUE2);
  Delay_ms(20);
  DAC_SetChannel2Data(DAC_Align_12b_R, APD_DAC2_80V);//APD_DAC2_VALUE3 - 80V  
}

void set_apd_voltage(float new_voltage)
{
  float dac_voltage = 0.0f;
  float tmp1 = DCDC_VREF * (1 + (DCDC_R_UP / DCDC_R_DOWN));
  
  dac_voltage = DCDC_VREF - (new_voltage * APD_CORR_COEF - tmp1) * (DCDC_R_DAC / DCDC_R_UP);
  
  float dac_value = dac_voltage * DAC_MAXIUM / AREF_VOLTAGE;
  DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)dac_value);
  APD_current_voltage = new_voltage;
}

void init_sys_clock(void)
{
  ErrorStatus HSEStartUpStatus;
  
  //Настраиваем систему тактирования
  RCC_DeInit(); /* RCC system reset(for debug purpose) */
  RCC_HSEConfig(RCC_HSE_ON);/* Enable HSE */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();/* Wait till HSE is ready */

  if (HSEStartUpStatus == SUCCESS)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);/* Enable Prefetch Buffer */
    FLASH_SetLatency(FLASH_Latency_2);/* Flash 2 wait state */
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);/* HCLK = SYSCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);/* PCLK2 = HCLK */
    RCC_PCLK1Config(RCC_HCLK_Div1);/* PCLK1 = HCLK */
    RCC_PREDIV1Config(RCC_PREDIV1_Source_HSE, RCC_PREDIV1_Div1);//12mhz
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_2);/* PLLCLK = 12MHz * 2 = 24 MHz <<<<<<<<<<<<<<<<<<<<<<<<<<*/
    RCC_PLLCmd(ENABLE);/* Enable PLL */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};/* Wait till PLL is ready */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);/* Select PLL as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08){}/* Wait till PLL is used as system clock source */
  }

}

void init_uart1(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  /* Enable GPIOA clock                                                   */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  
  // Configure USART1 Rx 
  GPIO_InitStructure.GPIO_Pin   = UART_RX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(UART_PORT, &GPIO_InitStructure);
  
 // Configure USART1 Tx 
  GPIO_InitStructure.GPIO_Pin   = UART_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(UART_PORT, &GPIO_InitStructure);
  
  GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
  
  USART_InitStructure.USART_BaudRate = 256000;   
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;   
  USART_InitStructure.USART_StopBits = USART_StopBits_1;   
  USART_InitStructure.USART_Parity = USART_Parity_No ;   
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  NVIC_EnableIRQ(USART1_IRQn);
  
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  
}

//init ADC for single software triggered measure
void init_adc_single_measure(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);//12 mhz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO,ENABLE);
  
  ADC_DeInit(ADC1);
  ADC_StructInit(&ADC_InitStructure);

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_DMACmd(ADC1, DISABLE);  
  ADC_Cmd(ADC1, ENABLE);
  
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1)){}; 
  ADC_StartCalibration(ADC1);   
  while(ADC_GetCalibrationStatus(ADC1)){};
}


uint16_t readADC1(uint8_t channel)
{
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_41Cycles5);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  return ADC_GetConversionValue(ADC1);
}


//do single measurements like temperature, battery voltage ...
void do_single_adc_measurements(void)
{
  init_adc_single_measure();
  APD_temperature_raw = readADC1(ADC_TEMP_CHANNEL);
  
  //init for main signal capture
  prepare_capture();
}



void i2c_init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(PLL_I2C_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);
  
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  //GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);//REMAP I2C1
  
  I2C_DeInit(PLL_I2C);
  I2C_StructInit(&I2C_InitStructure);
  
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 400000;
  
  I2C_Cmd(PLL_I2C, ENABLE);
  I2C_Init(PLL_I2C, &I2C_InitStructure);
  
  if (I2C_GetFlagStatus(PLL_I2C,I2C_FLAG_BUSY)) {
    // Reset the I2C block
    I2C_SoftwareResetCmd(PLL_I2C, ENABLE);
    I2C_SoftwareResetCmd(PLL_I2C, DISABLE);
  }
}

//Switching APD voltage by signal level - AGC
#ifdef MODULE_701A

void auto_switch_apd_voltage(uint16_t current_amplitude)
{
  set_apd_voltage(APD_SHIGH_VOLTAGE);//testing only
}

#else

void auto_switch_apd_voltage(uint16_t current_amplitude)
{
  if (APD_current_voltage < (APD_LOW_VOLTAGE + 2.0f))
  {
    //LOW APD voltage now
    if (current_amplitude < 3) return;//APD overload!
    if (current_amplitude < 150) 
      set_apd_voltage(APD_HIGH_VOLTAGE);//try to increase voltage
  }
  else
  {
    //HIGH APD voltage now
    if (current_amplitude > 2200) 
      set_apd_voltage(APD_LOW_VOLTAGE);//try to decrease voltage
  }
}

#endif


