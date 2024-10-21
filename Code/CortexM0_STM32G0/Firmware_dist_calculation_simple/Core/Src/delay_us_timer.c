#include "stdint.h"
#include "stm32g0xx.h"
#include "delay_us_timer.h"
#include "config_periph.h"

void dwt_init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  
  DELAY_US_TIMER_RCC_CMD(DELAY_US_TIMER_RCC);
  
  LL_TIM_StructInit(&TIM_InitStruct);
  
  // TIM1 configuration
  LL_TIM_DeInit(DELAY_US_TIMER_NAME);
  
  TIM_InitStruct.Prescaler = (uint16_t)(DELAY_US_TIMER_PRESCALER - 1);
  TIM_InitStruct.Autoreload = 0xFFFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(DELAY_US_TIMER_NAME, &TIM_InitStruct);
  LL_TIM_EnableCounter(DELAY_US_TIMER_NAME);
}

void dwt_delay(uint16_t us)
{
  if (us == 0)
    return;
  else if (us == 0xFFFF)
    us = 0xFFFF - 1;
  
  uint16_t start_time = DELAY_US_TIMER_NAME->CNT;
  uint16_t end_time  = start_time + us;
  
  if (end_time < start_time) //owerflow
  {
    //wait for overflow
    while (DELAY_US_TIMER_NAME->CNT < start_time) {}
  }
  
  while (DELAY_US_TIMER_NAME->CNT < end_time) {}
}

uint16_t get_dwt_value(void)
{
  return DELAY_US_TIMER_NAME->CNT;
}

