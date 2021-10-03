#include "stm32f0xx_i2c.h"
#include "i2c_functions.h"
#include "stm32f0xx_it.h"
#include "config_periph.h"

uint16_t i2c_good_cnt = 0;
uint16_t i2c_bad_cnt = 0;

#define I2C_TIMEOUT_MS  10

ErrorStatus i2c_start_transmission(uint8_t slaveAddress);
void i2c_end_transmission(void);
ErrorStatus I2C1_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
ErrorStatus I2C1_Write_NBytesDir(uint8_t driver_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);

void init_i2c(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(PLL_I2C_CLK, ENABLE);
  
      // Reset the I2C block
    I2C_SoftwareResetCmd(PLL_I2C);
  
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(I2C_GPIO_PORT, I2C_SDA_AF_SRC, I2C_GPIO_AF);//i2c
  GPIO_PinAFConfig(I2C_GPIO_PORT, I2C_SCL_AF_SRC, I2C_GPIO_AF);//i2c
  
  I2C_DeInit(PLL_I2C);
  I2C_StructInit(&I2C_InitStructure);
  
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = 0x0000020B;//from cube - 400K
  //I2C_InitStructure.I2C_Timing = 0x20000215;//from cube - 100K
  I2C_Init(PLL_I2C, &I2C_InitStructure);
  I2C_Cmd(PLL_I2C, ENABLE);
  
  /*
  if (I2C_GetFlagStatus(PLL_I2C,I2C_FLAG_BUSY)) 
  {
  }
  */
}

void PLL_send_data(uint8_t* data, uint16_t length)
{
  I2C1_Write_NBytesDir(PLL_I2C_ADDRESS, length, data);
}

//Send data to PLL
//start_reg - address
void PLL_send_data2(uint8_t start_reg, uint8_t* data, uint16_t length)
{
  I2C1_Write_NBytes(PLL_I2C_ADDRESS, start_reg, length, data);
}


//Write array of bytes
ErrorStatus I2C1_Write_NBytesDir(uint8_t driver_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
{
  uint8_t write_Num;
  
  uint32_t start_time = HAL_GetTick();
  
  while((I2C_GetFlagStatus(PLL_I2C, I2C_ISR_BUSY) != RESET) && 
        ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
  
  if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    return ERROR;
  }
  
  I2C_TransferHandling(I2C1, driver_Addr, number_Bytes, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
  
  for(write_Num = 0; write_Num < number_Bytes; write_Num++)
  {
    while((I2C_GetFlagStatus(PLL_I2C, I2C_FLAG_TXIS) == RESET) && 
          ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
    if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
    {
      i2c_bad_cnt++;
      return ERROR;
    }
    
    I2C_SendData(I2C1, write_Buffer[write_Num]);
  }
  
  while((I2C_GetFlagStatus(PLL_I2C, I2C_FLAG_STOPF) == RESET) && 
        ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
  if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    return ERROR;
  }
  
  i2c_good_cnt++;
  return SUCCESS;
}

//Write array of bytes, with given value of start register
ErrorStatus I2C1_Write_NBytes(
  uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
{
  uint8_t write_Num;
  
  uint32_t start_time = HAL_GetTick();
  
  while((I2C_GetFlagStatus(PLL_I2C, I2C_ISR_BUSY) != RESET) && 
        ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
  
  if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    return ERROR;
  }
  
  I2C_TransferHandling(I2C1, driver_Addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
  
  while((I2C_GetFlagStatus(PLL_I2C, I2C_FLAG_TXIS) == RESET) && 
        ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
  if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    return ERROR;
  }
  
  I2C_SendData(I2C1, start_Addr);
  
  while((I2C_GetFlagStatus(PLL_I2C, I2C_FLAG_TCR) == RESET) && 
        ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
  if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    return ERROR;
  }
  
  I2C_TransferHandling(I2C1, driver_Addr, number_Bytes, I2C_AutoEnd_Mode, I2C_No_StartStop);
  
  for(write_Num = 0; write_Num < number_Bytes; write_Num++)
  {
    while((I2C_GetFlagStatus(PLL_I2C, I2C_FLAG_TXIS) == RESET) && 
          ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
    if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
    {
      i2c_bad_cnt++;
      return ERROR;
    }
    
    I2C_SendData(I2C1, write_Buffer[write_Num]);
  }

  while((I2C_GetFlagStatus(PLL_I2C, I2C_FLAG_STOPF) == RESET) && 
        ((HAL_GetTick() - start_time) < I2C_TIMEOUT_MS)); 
  if ((HAL_GetTick() - start_time) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    return ERROR;
  }
  
  i2c_good_cnt++;
  return SUCCESS;
}

