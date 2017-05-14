#include "stm32f10x_i2c.h"
#include "i2c_functions.h"
#include "stm32f10x_it.h"
#include "config_periph.h"


uint8_t i2c_err_flag = 0;

//check with timeout
ErrorStatus I2C_CheckEventT(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
  uint32_t start_time = HAL_GetTick();
  while ((I2C_CheckEvent(I2Cx,I2C_EVENT) == ERROR) &&  ((HAL_GetTick() - start_time) < 5)){}
  if ((HAL_GetTick() - start_time) >= 5) i2c_err_flag = 1;
  return 	SUCCESS;
  //return I2C_CheckEventT(I2Cx,I2C_EVENT);
}

void I2C_WriteData(uint8_t data)
{
  // Просто вызываем готовую функцию из SPL и ждем, пока данные улетят
  I2C_SendData(PLL_I2C, data);
  while(!I2C_CheckEventT(PLL_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void I2C_EndTransmission(void)
{
  I2C_GenerateSTOP(PLL_I2C, ENABLE);
}

void I2C_StartTransmission(uint8_t transmissionDirection, uint8_t slaveAddress)
{
  i2c_err_flag = 0;
  uint32_t start_time = HAL_GetTick();
  while(I2C_GetFlagStatus(PLL_I2C, I2C_FLAG_BUSY) && ((HAL_GetTick() - start_time) < 5));// На всякий случай ждем, пока шина осовободится
  I2C_GenerateSTART(PLL_I2C, ENABLE);// Генерируем старт
  while(!I2C_CheckEventT(PLL_I2C, I2C_EVENT_MASTER_MODE_SELECT));// Ждем, пока взлетит нужный флаг
  I2C_Send7bitAddress(PLL_I2C, slaveAddress, transmissionDirection);
  if(transmissionDirection== I2C_Direction_Transmitter)
  {
    while(!I2C_CheckEventT(PLL_I2C,	I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  }
  else
  {
    while(!I2C_CheckEventT(PLL_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  }
  //i2c_auto_reboot();
}

void PLL_send_data(uint8_t* data, uint16_t length)
{
  uint16_t i;
  I2C_StartTransmission(I2C_Direction_Transmitter, PLL_I2C_ADDRESS);
  for (i=0;i<length;i++)
  {
    I2C_WriteData(data[i]);
  }
  I2C_EndTransmission();
}