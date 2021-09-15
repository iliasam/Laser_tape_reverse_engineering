#include "stm32f0xx.h"
#ifndef _I2C_FUNC
#define _I2C_FUNC

ErrorStatus I2C_CheckEventT(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
void I2C_WriteData(uint8_t data);
void i2c_auto_reboot(void);

void PLL_send_data(uint8_t* data, uint16_t length);
void PLL_send_data2(uint8_t start_reg, uint8_t* data, uint16_t length);

#endif
