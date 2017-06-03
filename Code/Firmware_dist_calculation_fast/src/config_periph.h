#ifndef _CONFIG_PEIRIPH_H
#define _CONFIG_PEIRIPH_H

#define LASER_DAC1_VALUE (uint16_t)(1737) //1.4v/3.3v*4096

#define APD_DAC2_VALUE1 (uint16_t)(2010) //1.62v/3.3v*4096
#define APD_DAC2_VALUE2 (uint16_t)(1588) //1.28v/3.3v*4096
#define APD_DAC2_80V (uint16_t)(1663) //1.34v/3.3v*4096 - ~~81V
#define APD_DAC2_93V (uint16_t)(1514) //1.22v/3.3v*4096 - ~~93V
#define APD_DAC2_98V (uint16_t)(1450) //~~95V
//#define APD_DAC2_98V (uint16_t)(1400) //~~102V - not calibrated



//#define PLL_I2C_ADDRESS         (0xC0>>1)
#define PLL_I2C_ADDRESS         0xC0


#define ADC_REF_CHANNEL         ADC_Channel_7 //PA7
#define ADC_SIGNAL_LOW          ADC_Channel_6 //PA6
#define ADC_SIGNAL_HIGH         ADC_Channel_3 //PA3

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
void uart_dma_init(void);
void uart_dma_start_tx(char* pointer, uint16_t length);

void init_adc_single_measure(void);
void do_single_adc_measurements(void);
void init_dac(void);
void i2c_init(void);

void enable_laser(void);
void disable_laser(void);

void start_apd_voltage(void);

#endif
