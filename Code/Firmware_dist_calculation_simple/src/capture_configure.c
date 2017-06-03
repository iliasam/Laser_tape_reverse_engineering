#include "stm32f10x_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "analyse.h"

volatile uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];//signal+reference points

extern uint8_t capture_done;

extern uint16_t APD_temperature_raw;
extern uint8_t  APD_current_voltage;

void init_adc_capture_timer(void);
void init_adc_capture(void);
void init_adc_capture_dma(void);
void init_adc_capture_timer(void);


void prepare_capture(void)
{
  init_adc_capture_timer();
  init_adc_capture();
  init_adc_capture_dma();
}

void init_adc_capture_timer(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  // TIM1 configuration
  TIM_DeInit(TIM1);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = TIMER1_DIV-1;
  TIM_TimeBaseStructure.TIM_Period = TIMER1_PERIOD-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  // Output Compare Inactive Mode configuration: Channel1 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;                
  TIM_OCInitStructure.TIM_Pulse = TIMER1_PERIOD/2; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;         
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
}

//configure ADC for capture signal from APD and reference signal
void init_adc_capture(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);//12 mhz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO,ENABLE);
  
  ADC_DeInit(ADC1);
  ADC_StructInit(&ADC_InitStructure);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  //ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;//scan - enable
  
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//repeat endless
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_SIGNAL_HIGH, 1, ADC_SampleTime_13Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_REF_CHANNEL, 2, ADC_SampleTime_13Cycles5);
  
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  
  ADC_DMACmd(ADC1, ENABLE);  
  ADC_Cmd(ADC1, ENABLE);
  
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1)){}; 
  ADC_StartCalibration(ADC1);   
  while(ADC_GetCalibrationStatus(ADC1)){};
}

void init_adc_capture_dma(void)
{
  DMA_InitTypeDef           DMA_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adc_capture_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ADC_CAPURE_BUF_LENGTH;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  //DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure); 
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);//transfer complete
  //DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);//half-transfer complete
  
  NVIC_SetPriority(DMA1_Channel1_IRQn, 15);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void start_adc_capture(void)
{
  TIM_Cmd(TIM1, DISABLE);
  ADC_Cmd(ADC1, DISABLE);
  
  DMA_Cmd(DMA1_Channel1, DISABLE);
  DMA1_Channel1->CMAR = (uint32_t)&adc_capture_buffer[0];
  DMA1_Channel1->CNDTR = ADC_CAPURE_BUF_LENGTH;
  DMA_ClearITPendingBit(DMA1_IT_HT1);
  DMA_ClearITPendingBit(DMA1_IT_TC1);
  DMA_Cmd(DMA1_Channel1, ENABLE);
  capture_done = 0;
  ADC_Cmd(ADC1, ENABLE);
  TIM_Cmd(TIM1, ENABLE);
}



void switch_apd_voltage(uint8_t new_voltage)
{
  switch (new_voltage)
  {
    case 82:
    {
      DAC_SetChannel2Data(DAC_Align_12b_R, APD_DAC2_80V);
      APD_current_voltage = new_voltage;
      break;
    }
    
    case 95:
    {
      DAC_SetChannel2Data(DAC_Align_12b_R, APD_DAC2_95V);
      APD_current_voltage = new_voltage;
      break;
    }
    
    default: break;
  }
  
}


