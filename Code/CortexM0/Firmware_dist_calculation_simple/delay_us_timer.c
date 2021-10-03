#include "stdint.h"
#include "stm32f0xx.h"
#include "delay_us_timer.h"
#include "config_periph.h"

void dwt_init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  DELAY_US_TIMER_RCC_CMD(DELAY_US_TIMER_RCC, ENABLE);
  
  TIM_DeInit(DELAY_US_TIMER_NAME);
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(DELAY_US_TIMER_PRESCALER - 1); 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(DELAY_US_TIMER_NAME, &TIM_TimeBaseStructure);
  TIM_Cmd(DELAY_US_TIMER_NAME, ENABLE);
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

