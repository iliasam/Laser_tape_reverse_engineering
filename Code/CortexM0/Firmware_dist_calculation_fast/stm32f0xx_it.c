/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "stm32f0xx.h"
#include "config_periph.h"
#include "measure_functions.h"
#include "main.h"
    
volatile uint8_t capture_done = 0;
volatile uint32_t uwTick;//taken from HAL

extern volatile dma_state_type dma_state;

void USART1_IRQHandler(void);

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
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
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles DMA interrupt request. Connected to ADC
  * @param  None
  * @retval : None
  */
void DMA1_Channel1_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_IT_HT1))
  {
    /* Clear DMA1 Channel1 half transfer interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_HT1);
  }
  else  if(DMA_GetITStatus(DMA1_IT_TC1))
  {
    /* Clear DMA1 Channel1 transfer complete interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    capture_done = 1;
    
    if (dma_state == DMA_FREQ1_CAPTURING) 
      dma_state = DMA_FREQ1_DONE;
    else if (dma_state == DMA_FREQ2_CAPTURING) 
      dma_state = DMA_FREQ2_DONE;
    else if (dma_state == DMA_FREQ3_CAPTURING) 
      dma_state = DMA_FREQ3_DONE;
    else 
      dma_state = DMA_NO_DATA;//error
  }
  else
  if(DMA_GetITStatus(DMA1_IT_TE1))
  {
    /* Clear DMA1 Channel1 transfer error interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_TE1);
  }
}

void USART1_IRQHandler(void)
{
  uint8_t rx_byte;
  if(USART_GetITStatus(UART_NAME, USART_IT_RXNE) != RESET)
  {
    USART_ClearFlag(UART_NAME, USART_IT_RXNE);
    rx_byte = USART_ReceiveData(UART_NAME);
    process_rx_data(rx_byte);
  }
}


//Taken from HAL
//######################################################
uint32_t HAL_GetTick(void)
{
  return uwTick;
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/