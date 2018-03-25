#ifndef _CONFIG_PEIRIPH_H
#define _CONFIG_PEIRIPH_H

#include "main.h"

#define LASER_DAC1_VALUE (uint16_t)(1737) //1.4v/3.3v*4096

#define APD_DAC2_VALUE1 (uint16_t)(2010) //1.62v/3.3v*4096
#define APD_DAC2_VALUE2 (uint16_t)(1588) //1.28v/3.3v*4096
#define APD_DAC2_80V (uint16_t)(1663) //1.34v/3.3v*4096 - ~~81V
#define APD_DAC2_93V (uint16_t)(1514) //1.22v/3.3v*4096 - ~~93V
#define APD_DAC2_95V (uint16_t)(1450) //~~95V

#define APD_LOW_VOLTAGE         80.6f
#define APD_HIGH_VOLTAGE        98.6f

#define DCDC_R_UP               5100000.0f //Ohm - upper - R37
#define DCDC_R_DOWN             74000.0f //Ohm - down - R103
#define DCDC_R_DAC              (float)(2100 + 46800) //Ohm - after dac - R38 + R39
#define DCDC_VREF               1.27f //V - internal VREF voltage of the DC-DC

#ifdef MODULE_701A
  #define AREF_VOLTAGE            3.36f //AREF voltage of the MCU
  #define ADC_SIGNAL              ADC_Channel_6 //PA6
  #define APD_CORR_COEF           1.0236f
  #define APD_SHIGH_VOLTAGE       115.0f
#else
  #define AREF_VOLTAGE            3.32f //AREF voltage of the MCU
  #define ADC_SIGNAL              ADC_Channel_3 //PA3
  #define APD_CORR_COEF           1.0f
  #define APD_SHIGH_VOLTAGE       105.0f
#endif

#define DAC_MAXIUM              4095.0f

#define PLL_I2C_ADDRESS         0xC0

#define ADC_REF_CHANNEL         ADC_Channel_7 //PA7

#define ADC_TEMP_CHANNEL        ADC_Channel_8 //PB0

#define ADC_REF_PIN             GPIO_Pin_7
#define ADC_SIGNAL_LOW_PIN      GPIO_Pin_6
#define ADC_SIGNAL_HIGH_PIN     GPIO_Pin_3

#define UART_RX_PIN             GPIO_Pin_7
#define UART_TX_PIN             GPIO_Pin_6
#define UART_PORT               GPIOB

#define PLL_I2C                 I2C2
#define PLL_I2C_CLK             RCC_APB1Periph_I2C2

#define I2C_SDA_PIN             GPIO_Pin_11
#define I2C_SCL_PIN             GPIO_Pin_10
#define I2C_GPIO_PORT           GPIOB

#define BEEP_PIN                GPIO_Pin_14
#define BEEP_PORT               GPIOB

#define LASER_POWER_PIN         GPIO_Pin_2 //1 - disable laser
#define LASER_PORT              GPIOA

#define MAIN_POWER_PIN          GPIO_Pin_15//0 - enable main DC-DC
#define MAIN_POWER_PORT         GPIOC

#define ANALOG_POWER_PIN        GPIO_Pin_12//0 - enable analog power (APD, PLL)
#define ANALOG_POWER_PORT       GPIOB

void init_all_hardware(void);
void init_gpio(void);
void init_sys_clock(void);
void init_uart1(void);
void init_adc_single_measure(void);
void do_single_adc_measurements(void);
void init_dac(void);
void i2c_init(void);

void enable_laser(void);
void disable_laser(void);

void start_apd_voltage(void);
void set_apd_voltage(float new_voltage);
void auto_switch_apd_voltage(uint16_t current_amplitude);

#endif
