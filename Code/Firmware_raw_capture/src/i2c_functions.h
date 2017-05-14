#include "stm32f10x.h"
#ifndef _I2C_FUNC
#define _I2C_FUNC

ErrorStatus I2C_CheckEventT(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
void I2C_WriteData(uint8_t data);
void I2C_StartTransmission(uint8_t transmissionDirection, uint8_t slaveAddress);
void I2C_EndTransmission(void);
void i2c_auto_reboot(void);

void PLL_send_data(uint8_t* data, uint16_t length);

#endif
