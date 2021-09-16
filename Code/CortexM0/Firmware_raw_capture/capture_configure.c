#include "stm32f0xx_it.h"
#include "config_periph.h"
#include "capture_configure.h"

/* Private typedef -----------------------------------------------------------*/

uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];//signal+reference points

/* Private variables ---------------------------------------------------------*/
uint16_t signal_buffer[ADC_CAPURE_BUF_LENGTH/2];
uint16_t reference_buffer[ADC_CAPURE_BUF_LENGTH/2];

extern uint8_t capture_done;

/* Private function prototypes -----------------------------------------------*/
void init_adc_capture_timer(void);
void init_adc_capture(void);
void init_adc_capture_dma(void);
void init_adc_capture_timer(void);


/* Private functions ---------------------------------------------------------*/

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
  TIM_TimeBaseStructure.TIM_Prescaler = TIMER1_DIV - 1;
  TIM_TimeBaseStructure.TIM_Period = TIMER1_PERIOD - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  // Output Compare Inactive Mode configuration: Channel1 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  // TIM_OutputState_Enable needed for F030 to be ADC trigger
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = TIMER1_PERIOD / 2; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;         
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//??
  
  // needed for F030 to be ADC trigger
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void init_adc_capture(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  
  //14 MHz is max for F0 ADC
  //PCLK is 48 MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);//48 / 4 = 12
  
  ADC_StructInit(&ADC_InitStructure);
  
  /* ADC1 configuration ------------------------------------------------------*/
  
    // ADC1 configuration  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//repeat endless
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;//see chanels values
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_ChannelConfig(ADC1, ADC_SIGNAL | ADC_REF_CHANNEL, ADC_SampleTime_1_5Cycles);

  ADC_GetCalibrationFactor(ADC1);
  
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_OneShot);
  ADC_DMACmd(ADC1, ENABLE);//adc generate events for DMA
  
  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
  
  ADC_StartOfConversion(ADC1);
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
  
  NVIC_SetPriority(DMA1_Channel1_IRQn, 15);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void start_adc_capture(void)
{
  //Start TIM1 -> ADC -> DMA -> RAM
  
  TIM_Cmd(TIM1, DISABLE);
  ADC_Cmd(ADC1, DISABLE);
  
  DMA_Cmd(DMA1_Channel1, DISABLE);
  DMA1_Channel1->CMAR = (uint32_t)adc_capture_buffer;
  DMA1_Channel1->CNDTR = ADC_CAPURE_BUF_LENGTH;
  DMA_Cmd(DMA1_Channel1, ENABLE);
  capture_done = 0;
  
  ADC_Cmd(ADC1, ENABLE);
  ADC_StartOfConversion(ADC1);
  TIM_Cmd(TIM1, ENABLE);
}

// split captured data from "adc_capture_buffer" 
// to "reference_buffer" and "signal_buffer"
void sort_captured_data(void)
{
  uint16_t i;
  uint16_t count = 0;
  for (i=0; i < ADC_CAPURE_BUF_LENGTH; i+=2)
  {
    reference_buffer[count] = adc_capture_buffer[i];
    signal_buffer[count] = adc_capture_buffer[i+1];
    count++;
  }
}