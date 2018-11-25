/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "config_periph.h"
#include "measure_functions.h"

volatile uint32_t uwTick;//taken from HAL

volatile uint8_t capture_done = 0;//show that DMA done transfer
extern volatile dma_state_type dma_state;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  uwTick++;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval : None
  */
void DMA1_Channel1_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC1)) // Test on DMA1 Channel1 transfer complete interrupt 
  {
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    capture_done = 1;
    //GPIO_ResetBits(KEY_3_PORT, KEY_3_PIN);
    
    if (dma_state == DMA_FREQ1_CAPTURING) dma_state = DMA_FREQ1_DONE;
    else if (dma_state == DMA_FREQ2_CAPTURING) dma_state = DMA_FREQ2_DONE;
    else if (dma_state == DMA_FREQ3_CAPTURING) dma_state = DMA_FREQ3_DONE;
    else dma_state = DMA_NO_DATA;//error
  }
  else if (DMA_GetITStatus(DMA1_IT_HT1))// Test on DMA1 Channel1 half transfer interrupt */
  {
    /* Clear DMA1 Channel1 half transfer interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_HT1);
  }
  else
  /* Test on DMA1 Channel1 transfer error interrupt */
  if(DMA_GetITStatus(DMA1_IT_TE1))
  {
    /* Clear DMA1 Channel1 transfer error interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_TE1);
  }
  
}

void USART1_IRQHandler(void)
{
  uint8_t rx_byte;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    USART_ClearFlag(USART1, USART_IT_RXNE);
    rx_byte = USART_ReceiveData(USART1);
    process_rx_data(rx_byte);
  }
}





/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM2_IRQHandler(void)
{
  TIM2->SR &= ~TIM_SR_UIF;//сбросить флаг прерывания
       
}

//Taken from HAL
//######################################################
uint32_t HAL_GetTick(void)
{
  return uwTick;
}

void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay)
  {
  }
}


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
