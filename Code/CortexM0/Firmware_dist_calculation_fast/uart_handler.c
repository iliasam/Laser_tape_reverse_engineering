#include "main.h"
#include "uart_handler.h"
#include "string.h"
#include "config_periph.h"

uint8_t uart_disabled_flag = 0;

void uart_dma_init(void);

void init_uart1(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  // Enable GPIO clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_USARTCLKConfig(RCC_USART1CLK_SYSCLK);
  
  // Configure USART1 Rx as alternate function push-pull
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = UART_RX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//to detect external pullup
  GPIO_Init(UART_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(UART_PORT, UART_RX_PIN_SRC, UART_GPIO_AF); //GPIO_AF_1 - uart1
  
#if defined(DETECT_UART_PRESENCE)
  if ((UART_PORT->IDR & UART_RX_PIN) == 0)
  {
    //no external pull up, debugger mode
    uart_disabled_flag = 1;
    return;
  }
#endif
  
  uart_dma_init();
  
  GPIO_InitStructure.GPIO_Pin = UART_TX_PIN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(UART_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(UART_PORT, UART_TX_PIN_SRC, UART_GPIO_AF); //GPIO_AF_1 - uart1
  
  USART_InitStructure.USART_BaudRate    = UART_BAURDATE;
  USART_InitStructure.USART_WordLength  = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits    = USART_StopBits_1;
  USART_InitStructure.USART_Parity      = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode        = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_ITConfig(UART_NAME, USART_IT_RXNE, ENABLE);
  NVIC_SetPriority(UART_IRQ_NAME, 10);
  NVIC_EnableIRQ(UART_IRQ_NAME);
  
  //TX is by DMA
  USART_DMACmd(UART_NAME, USART_DMAReq_Tx, ENABLE);
  
  USART_Init(UART_NAME, &USART_InitStructure);
  USART_Cmd(UART_NAME, ENABLE);
}

//Initialize DMA for UART
void uart_dma_init(void)
{
  DMA_InitTypeDef           DMA_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  DMA_DeInit(UART_TX_DMA_CHANNEL);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART_NAME->TDR;
  DMA_InitStructure.DMA_MemoryBaseAddr     = SRAM_BASE;//only for init
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize         = 10;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  
  DMA_RemapConfig(DMA1, DMA1_CH2_USART1_TX);
  DMA_Init(UART_TX_DMA_CHANNEL, &DMA_InitStructure);
}

//Prepare UART DMA for transmit data and start TX
void uart_dma_start_tx(uint8_t* data_ptr, uint16_t length)
{ 
  if (uart_disabled_flag)
    return;
  
  DMA_Cmd(UART_TX_DMA_CHANNEL, DISABLE);
  UART_TX_DMA_CHANNEL->CMAR = (uint32_t)(data_ptr);
  UART_TX_DMA_CHANNEL->CNDTR = (uint32_t)length;
  DMA_ClearFlag(UART_TX_DMA_FLAG);
  DMA_ClearFlag(UART_TX_DMA_FLAGE);
  DMA_Cmd(UART_TX_DMA_CHANNEL, ENABLE);
}

