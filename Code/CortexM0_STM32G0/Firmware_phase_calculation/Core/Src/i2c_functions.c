#include "stm32g0xx_ll_i2c.h"
#include "i2c_functions.h"
#include "stm32g0xx_it.h"
#include "config_periph.h"

uint16_t i2c_good_cnt = 0;
uint16_t i2c_bad_cnt = 0;

#define I2C_TIMEOUT_MS  10

ErrorStatus i2c_start_transmission(uint8_t slaveAddress);
void i2c_end_transmission(void);
ErrorStatus I2C1_Write_NBytes(
  uint8_t driver_addr, uint8_t start_addr, uint8_t number_bytes, uint8_t *write_buffer);
ErrorStatus I2C1_Write_NBytesDir(uint8_t driver_addr, uint8_t number_bytes, uint8_t *write_buffer);

extern volatile uint32_t ms_uptime;

void init_i2c(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_RCC_SetI2CClockSource(PLL_I2C_CLK);
  LL_IOP_GRP1_EnableClock(I2C_GPIO_PORT_CLK);
  
  GPIO_InitStruct.Pin = I2C_SCL_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = I2C_GPIO_AF;
  LL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = I2C_SDA_PIN;
  LL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
  
  LL_APB1_GRP1_EnableClock(PLL_I2C_CLK_2);
  
  LL_I2C_DeInit(PLL_I2C);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00602173; //from cube - 400K
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(PLL_I2C, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(PLL_I2C);
  LL_I2C_SetOwnAddress2(PLL_I2C, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(PLL_I2C);
  LL_I2C_DisableGeneralCall(PLL_I2C);
  LL_I2C_EnableClockStretching(PLL_I2C);
}

void pll_send_data(uint8_t* data, uint16_t length)
{
  I2C1_Write_NBytesDir(PLL_I2C_ADDRESS, length, data);
}

//Send data to PLL
//start_reg - address
void pll_send_data2(uint8_t start_reg, uint8_t* data, uint16_t length)
{
  I2C1_Write_NBytes(PLL_I2C_ADDRESS, start_reg, length, data);
}


//Write array of bytes
ErrorStatus I2C1_Write_NBytesDir(uint8_t driver_addr, uint8_t number_bytes, uint8_t *write_buffer)
{
  uint8_t write_num;
  
  uint32_t start_time_ms = ms_uptime;
  
  while((LL_I2C_IsActiveFlag_BUSY(PLL_I2C) != RESET) && 
        ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS)); 
  
  if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    LL_I2C_ClearFlag_STOP(PLL_I2C);
    return ERROR;
  }
  
  LL_I2C_HandleTransfer(PLL_I2C, driver_addr, LL_I2C_ADDRSLAVE_7BIT, 
    number_bytes, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  
  for(write_num = 0; write_num < number_bytes; write_num++)
  {
    while((LL_I2C_IsActiveFlag_TXIS(PLL_I2C) == RESET) && 
          ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS));
    
    if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
    {
      i2c_bad_cnt++;
      LL_I2C_ClearFlag_STOP(PLL_I2C);
      return ERROR;
    }
    
    LL_I2C_TransmitData8(PLL_I2C, write_buffer[write_num]);
  }
  
  
  while((LL_I2C_IsActiveFlag_STOP(PLL_I2C) == RESET) && 
    ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS));
  
  if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    LL_I2C_ClearFlag_STOP(PLL_I2C);
    return ERROR;
  }
  
  LL_I2C_ClearFlag_STOP(PLL_I2C);
  
  i2c_good_cnt++;
  
  return SUCCESS;
}

//Write array of bytes, with given value of start register
ErrorStatus I2C1_Write_NBytes(
  uint8_t driver_addr, uint8_t start_addr, uint8_t number_bytes, uint8_t *write_buffer)
{
  uint8_t write_num;
  
  uint32_t start_time_ms = ms_uptime;
  
  while((LL_I2C_IsActiveFlag_BUSY(PLL_I2C) != RESET) && 
        ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS)); 
  
  if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    LL_I2C_ClearFlag_STOP(PLL_I2C);
    return ERROR;
  }
  
  //Send "start_addr"
  LL_I2C_HandleTransfer(PLL_I2C, driver_addr, LL_I2C_ADDRSLAVE_7BIT, 
    1, LL_I2C_MODE_RELOAD, LL_I2C_GENERATE_START_WRITE);
  
  while((LL_I2C_IsActiveFlag_TXIS(PLL_I2C) == RESET) && 
        ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS)); 
  if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    return ERROR;
  }
  
  LL_I2C_TransmitData8(PLL_I2C, start_addr);
  
  while((LL_I2C_IsActiveFlag_TCR(PLL_I2C) == RESET) && 
        ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS)); 
  if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    LL_I2C_ClearFlag_STOP(PLL_I2C);
    return ERROR;
  }
  
  //Send "write_buffer" bytes
  LL_I2C_HandleTransfer(PLL_I2C, driver_addr, LL_I2C_ADDRSLAVE_7BIT, 
    number_bytes, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_NOSTARTSTOP);

  for(write_num = 0; write_num < number_bytes; write_num++)
  {
    while((LL_I2C_IsActiveFlag_TXIS(PLL_I2C) == RESET) && 
          ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS)); 
    if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
    {
      i2c_bad_cnt++;
      LL_I2C_ClearFlag_STOP(PLL_I2C);
      return ERROR;
    }
    
    LL_I2C_TransmitData8(PLL_I2C, write_buffer[write_num]);
  }

  while((LL_I2C_IsActiveFlag_STOP(PLL_I2C) == RESET) && 
        ((ms_uptime - start_time_ms) < I2C_TIMEOUT_MS));
  
  if ((ms_uptime - start_time_ms) >= I2C_TIMEOUT_MS)
  {
    i2c_bad_cnt++;
    LL_I2C_ClearFlag_STOP(PLL_I2C);
    return ERROR;
  }
  
  i2c_good_cnt++;
  
  return SUCCESS;
}

