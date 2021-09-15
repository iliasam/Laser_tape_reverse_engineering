#include "stm32f0xx_i2c.h"
#include "i2c_functions.h"
#include "stm32f0xx_it.h"
#include "config_periph.h"


uint8_t i2c_err_flag = 0;

void i2c_start_transmission(uint8_t slaveAddress);
void i2c_end_transmission(void);

//check with timeout
ErrorStatus I2C_CheckEventT(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
  /*
  uint32_t start_time = HAL_GetTick();
  while ((I2C_CheckEvent(I2Cx,I2C_EVENT) == ERROR) &&  ((HAL_GetTick() - start_time) < 5)){}
  if ((HAL_GetTick() - start_time) >= 5) i2c_err_flag = 1;
  */
  return SUCCESS;
}

void PLL_send_data(uint8_t* data, uint16_t length)
{
  uint16_t i;
  i2c_start_transmission(PLL_I2C_ADDRESS);
  for (i=0; i < length; i++)
  {
    I2C_WriteData(data[i]);
  }
  i2c_end_transmission();
}

//Send data to PLL
//start_reg - address
void PLL_send_data2(uint8_t start_reg, uint8_t* data, uint16_t length)
{
  uint16_t i;
  i2c_start_transmission(PLL_I2C_ADDRESS);
  I2C_WriteData(start_reg);
  for (i=0;i<length;i++)
  {
    I2C_WriteData(data[i]);
  }
  i2c_end_transmission();
}

void I2C_WriteData(uint8_t data)
{       
  //Just blocking TX
  I2C_SendData(PLL_I2C, data);
  while((I2C_GetFlagStatus(PLL_I2C, I2C_ISR_TCR) == RESET));
}

void i2c_end_transmission(void)
{
  I2C_GenerateSTOP(PLL_I2C, ENABLE);
  I2C_GenerateSTOP(PLL_I2C, DISABLE);
  I2C_ClearFlag(PLL_I2C, I2C_ICR_STOPCF);
}

void i2c_start_transmission(uint8_t slaveAddress)
{ 
  i2c_err_flag = 0;
  uint32_t start_time = HAL_GetTick();
  I2C_TransferHandling(PLL_I2C, slaveAddress, 1, 
                       I2C_Reload_Mode, I2C_Generate_Start_Write);
  
  while((I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET) && 
        ((HAL_GetTick() - start_time) < 5));
 
}

