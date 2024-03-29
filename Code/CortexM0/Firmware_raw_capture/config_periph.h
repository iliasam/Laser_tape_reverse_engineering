#ifndef _CONFIG_PEIRIPH_H
#define _CONFIG_PEIRIPH_H

#include "main.h"

// When enabled, RX pin state will be analyzed at startup
// If RX is low, UART is disabled. This is needed for debugger
//if not defined, UART is always enabled - "boot" configuration
#define DETECT_UART_PRESENCE

#define APD_LOW_VOLTAGE         80.6f
#define APD_HIGH_VOLTAGE        98.6f
#define APD_DEFAULT_VOLTAGE     100.6f
#define APD_SHIGH_VOLTAGE       115.0f

#define DCDC_R_UP               5100000.0f //Ohm - upper - R37
#define DCDC_R_DOWN             74000.0f //Ohm - down - R103
#define DCDC_R_DAC              (float)(2100 + 46800) //Ohm - after dac - R38 + R39
#define DCDC_VREF               1.27f //V - internal VREF voltage of the DC-DC

#define AREF_VOLTAGE            3.3f //AREF voltage of the MCU
#define ADC_SIGNAL              ADC_Channel_5 //PA5
#define APD_CORR_COEF           1.0236f

#define ADC_REF_CHANNEL         ADC_Channel_4 //PA4
#define ADC_REF_PIN             GPIO_Pin_4
#define ADC_SIGNAL_PIN          GPIO_Pin_5

#define PLL_I2C_ADDRESS         0xC0

#define UART_RX_PIN             GPIO_Pin_15
#define UART_TX_PIN             GPIO_Pin_14
#define UART_PORT               GPIOA
#define UART_GPIO_AF            GPIO_AF_1
#define UART_RX_PIN_SRC         GPIO_PinSource15
#define UART_TX_PIN_SRC         GPIO_PinSource14
#define UART_BAURDATE           256000

#define PLL_I2C                 I2C1
#define PLL_I2C_CLK             RCC_APB1Periph_I2C1

#define I2C_SDA_PIN             GPIO_Pin_10
#define I2C_SCL_PIN             GPIO_Pin_9
#define I2C_GPIO_PORT           GPIOA
#define I2C_SDA_AF_SRC          GPIO_PinSource10
#define I2C_SCL_AF_SRC          GPIO_PinSource9

#define I2C_GPIO_AF             GPIO_AF_4

#define BEEP_PIN                GPIO_Pin_7
#define BEEP_PORT               GPIOA


#define MAIN_POWER_PIN          GPIO_Pin_8//0 - enable main DC-DC
#define MAIN_POWER_PORT         GPIOA

#define ANALOG_POWER_PIN        GPIO_Pin_1//0 - enable analog power (APD, PLL)
#define ANALOG_POWER_PORT       GPIOF

// PWM for APD and laser controlling

#define VOLT_PWM_TIMER_RCC_CMD          RCC_APB1PeriphClockCmd
#define VOLT_PWM_TIMER_RCC              RCC_APB1Periph_TIM2
#define VOLT_PWM_TIMER_NAME             TIM2
#define VOLT_PWM_TIMER_PRESCALER        (0)
#define VOLT_PWM_TIMER_PERIOD           (480) //48M / 480 = 100khz

//MCU_APD_CTRL 
#define VOLT_PWM_TIMER_APD_CTRL_PIN     GPIO_Pin_1
#define VOLT_PWM_TIMER_APD_CTRL_PORT    GPIOA
#define VOLT_PWM_TIMER_APD_CTRL_SRC     GPIO_PinSource1
#define VOLT_PWM_TIMER_APD_CTRL_AF      GPIO_AF_2

//LASER_POWER_PWM
#define VOLT_PWM_TIMER_LASER_CTRL_PIN   GPIO_Pin_2
#define VOLT_PWM_TIMER_LASER_CTRL_PORT  GPIOA
#define VOLT_PWM_TIMER_LASER_CTRL_SRC   GPIO_PinSource2
#define VOLT_PWM_TIMER_LASER_CTRL_AF    GPIO_AF_2

//Measured at original FW
#define VOLT_PWM_TIMER_LASER_DUTY       (uint16_t)(0.4f * VOLT_PWM_TIMER_PERIOD)


void enable_laser(void);
void disable_laser(void);
void set_apd_voltage(float new_voltage);
void init_all_hardware(void);
void delay_ms(uint32_t ms);

#endif
