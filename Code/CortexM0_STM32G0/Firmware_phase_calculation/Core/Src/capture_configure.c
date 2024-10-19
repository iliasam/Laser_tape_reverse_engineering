#include "stm32g0xx_it.h"
#include "config_periph.h"
#include "capture_configure.h"
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
int16_t phase_buffer[REPEAT_NUMBER];
uint16_t adc_capture_buffer[ADC_CAPURE_BUF_LENGTH];//signal+reference points

/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t capture_done;
extern uint16_t APD_temperature_raw;
extern float  APD_current_voltage;

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
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  
  LL_TIM_StructInit(&TIM_InitStruct);
  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
  
  // TIM1 configuration
  LL_TIM_DeInit(TIM1);
  
  TIM_InitStruct.Prescaler = TIMER1_DIV - 1;
  TIM_InitStruct.Autoreload = TIMER1_PERIOD - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  // TIM_OutputState_Enable needed for to be ADC trigger
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = TIMER1_PERIOD / 2;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;

  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  
  // needed to be ADC trigger
  LL_TIM_EnableAllOutputs(TIM1);
}

void init_adc_capture(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
  
  uint32_t Timeout = 1; //Variable used for Timeout management
  
  LL_ADC_CommonDeInit(__LL_ADC_COMMON_INSTANCE(ADC1));
  
  // ADC1 configuration ------------------------------------------------------
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_CH4;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD); //see chanels values
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_REG_SetSequencerChAdd(ADC1, ADC_REF_CHANNEL | ADC_SIGNAL_CHANNEL);
  
  
  // Poll for ADC channel configuration ready 
  while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
  {
    // Check Systick counter flag to decrement the time-out value 
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
        Error_Handler();
      }
    }
  }
  
  // Clear flag ADC channel configuration ready 
  LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  // Sample time is important!
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_7CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);

  // Enable ADC internal voltage regulator
  LL_ADC_EnableInternalRegulator(ADC1);
  delay_ms(1);
  
  LL_ADC_StartCalibration(ADC1);
  delay_ms(2);
  if (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
  {
    Error_Handler();
  }
  
  LL_ADC_Enable(ADC1);
  Timeout = 3;
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
  {
    // Check Systick counter flag to decrement the time-out value 
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
        Error_Handler();
      }
    }
  }
  
  //DMA transfer requests are stopped when number of DMA data transfers  is reached.
  //Enable DMA
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED);
}

void init_adc_capture_dma(void)
{

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
  
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         (uint32_t)&ADC1->DR,
                         (uint32_t)adc_capture_buffer,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_1,
                       ADC_CAPURE_BUF_LENGTH);
                       
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 15);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void start_adc_capture(void)
{
  //Start TIM1 -> ADC -> DMA -> RAM
  
  LL_TIM_DisableCounter(TIM1);
  //LL_ADC_Disable(ADC1);
  LL_ADC_REG_StopConversion(ADC1);
  
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         (uint32_t)&ADC1->DR,
                         (uint32_t)adc_capture_buffer,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_1,
                       ADC_CAPURE_BUF_LENGTH);
  
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  capture_done = 0;
  
  //ADC_Cmd(ADC1, ENABLE);
  LL_ADC_REG_StartConversion(ADC1);
  LL_TIM_EnableCounter(TIM1);
}

//for single frequency
AnalyseResultType do_capture(void)
{
  uint8_t i;
  AnalyseResultType main_result = {0,0};
  
  AnalyseResultType signal_result = {0,0};
  AnalyseResultType reference_result = {0,0};
  
  uint32_t amplitude_summ = 0;
  int16_t tmp_phase = 0;
  
  for (i = 0; i < REPEAT_NUMBER; i++)
  {
    start_adc_capture();
    while(capture_done == 0){}
    signal_result = goertzel_analyse(&adc_capture_buffer[0]);//signal
    reference_result = goertzel_analyse(&adc_capture_buffer[1]);//reference
    
    tmp_phase = reference_result.Phase - signal_result.Phase;//difference between signal and reference phase
    if (tmp_phase < 0) 
      tmp_phase = 360 * PHASE_MULT + tmp_phase;
    
    amplitude_summ += (uint32_t)signal_result.Amplitude;
    phase_buffer[i] = tmp_phase;
  }
  
  main_result.Amplitude = (uint16_t)(amplitude_summ / REPEAT_NUMBER);
  main_result.Phase = calculate_avr_phase(phase_buffer, REPEAT_NUMBER);

  main_result.Phase = calculate_corrected_phase(
    APD_temperature_raw, main_result.Amplitude, (uint8_t)APD_current_voltage, main_result.Phase);
  //phase < 0 or > 360
  if (main_result.Phase < 0) 
    main_result.Phase = MAX_ANGLE * PHASE_MULT + main_result.Phase;
  else if (main_result.Phase > (MAX_ANGLE * PHASE_MULT))
    main_result.Phase = main_result.Phase - MAX_ANGLE * PHASE_MULT;
  
  
  return main_result;
}
