#ifndef _CONFIG_PEIRIPH_H
#define _CONFIG_PEIRIPH_H

#include "main.h"

#define GPIO_Pin_0   LL_GPIO_PIN_0
#define GPIO_Pin_1   LL_GPIO_PIN_1
#define GPIO_Pin_2   LL_GPIO_PIN_2
#define GPIO_Pin_3   LL_GPIO_PIN_3
#define GPIO_Pin_4   LL_GPIO_PIN_4
#define GPIO_Pin_5   LL_GPIO_PIN_5
#define GPIO_Pin_6   LL_GPIO_PIN_6
#define GPIO_Pin_7   LL_GPIO_PIN_7
#define GPIO_Pin_8   LL_GPIO_PIN_8
#define GPIO_Pin_9   LL_GPIO_PIN_9
#define GPIO_Pin_10  LL_GPIO_PIN_10
#define GPIO_Pin_11  LL_GPIO_PIN_11
#define GPIO_Pin_12  LL_GPIO_PIN_12
#define GPIO_Pin_13  LL_GPIO_PIN_13
#define GPIO_Pin_14  LL_GPIO_PIN_14
#define GPIO_Pin_15  LL_GPIO_PIN_15



#define APD_LOW_VOLTAGE         80.6f
#define APD_HIGH_VOLTAGE        98.6f
#define APD_DEFAULT_VOLTAGE     103.6f
#define APD_SHIGH_VOLTAGE       115.0f

//Constants for "ENHANCED_APD_CALIBADION"
#define APD_START_CALIB_VOLTAGE 75.0f
#define APD_STOP_CALIB_VOLTAGE  123.0f
#define APD_VOLTAGE_RANGE       20.0f
#define APD_DEFAULT_SATURATIION_VOLT  114.0f

#define DCDC_R_UP               5100000.0f //Ohm - upper - R37
#define DCDC_R_DOWN             74000.0f //Ohm - down - R103
#define DCDC_R_DAC              (float)(2100 + 46800) //Ohm - after dac - R38 + R39
#define DCDC_VREF               1.27f //V - internal VREF voltage of the DC-DC

#define AREF_VOLTAGE            3.3f //AREF voltage of the MCU
#define ADC_SIGNAL_CHANNEL      LL_ADC_CHANNEL_5 //PA5
#define APD_CORR_COEF           1.0236f

#define ADC_REF_CHANNEL         LL_ADC_CHANNEL_4 //PA4
#define ADC_REF_PIN             GPIO_Pin_4
#define ADC_SIGNAL_PIN          GPIO_Pin_5

#define ADC_TEMP_CHANNEL        LL_ADC_CHANNEL_8 //PB8
#define ADC_TEMP_PIN            GPIO_Pin_0
#define ADC_TEMP_PORT           GPIOB

#define PLL_I2C_ADDRESS         0xC0

#define UART_RX_PIN             GPIO_Pin_10
#define UART_TX_PIN             GPIO_Pin_9
#define UART_PORT               GPIOA
#define UART_GPIO_AF            LL_GPIO_AF_1
#define UART_RX_PIN_SRC         GPIO_PinSource10
#define UART_TX_PIN_SRC         GPIO_PinSource9
#define UART_BAURDATE           256000

#define PLL_I2C                 I2C1
#define PLL_I2C_CLK             LL_RCC_I2C1_CLKSOURCE_PCLK1
#define PLL_I2C_CLK_2           LL_APB1_GRP1_PERIPH_I2C1

#define I2C_GPIO_PORT_CLK       LL_IOP_GRP1_PERIPH_GPIOB
#define I2C_SDA_PIN             GPIO_Pin_7
#define I2C_SCL_PIN             GPIO_Pin_6
#define I2C_GPIO_PORT           GPIOB

#define I2C_GPIO_AF             LL_GPIO_AF_6

#define BEEP_PIN                GPIO_Pin_7
#define BEEP_PORT               GPIOA


#define MAIN_POWER_PIN          GPIO_Pin_8//0 - enable main DC-DC
#define MAIN_POWER_PORT         GPIOA

#define ANALOG_POWER_PIN        GPIO_Pin_6//1 - enable analog power (APD)
#define ANALOG_POWER_PORT       GPIOC

// PWM for APD and laser controlling

#define VOLT_PWM_TIMER_RCC_CMD          LL_APB1_GRP1_EnableClock
#define VOLT_PWM_TIMER_RCC              LL_APB1_GRP1_PERIPH_TIM2
#define VOLT_PWM_TIMER_NAME             TIM2
#define VOLT_PWM_TIMER_PRESCALER        (0)
#define VOLT_PWM_TIMER_PERIOD           (640) //64M / 640 = 100khz

//MCU_APD_CTRL 
#define VOLT_PWM_TIMER_APD_CTRL_PIN     GPIO_Pin_1
#define VOLT_PWM_TIMER_APD_CTRL_PORT    GPIOA
#define VOLT_PWM_TIMER_APD_CTRL_SRC     GPIO_PinSource1
#define VOLT_PWM_TIMER_APD_CTRL_AF      LL_GPIO_AF_2

//LASER_POWER_PWM
#define VOLT_PWM_TIMER_LASER_CTRL_PIN   GPIO_Pin_2
#define VOLT_PWM_TIMER_LASER_CTRL_PORT  GPIOA
#define VOLT_PWM_TIMER_LASER_CTRL_SRC   GPIO_PinSource2
#define VOLT_PWM_TIMER_LASER_CTRL_AF    LL_GPIO_AF_2

//Measured at original FW
#define VOLT_PWM_TIMER_LASER_DUTY       (uint16_t)(0.5f * VOLT_PWM_TIMER_PERIOD)

#define DELAY_US_TIMER_RCC_CMD          LL_APB1_GRP1_EnableClock
#define DELAY_US_TIMER_RCC              LL_APB1_GRP1_PERIPH_TIM3
#define DELAY_US_TIMER_NAME             TIM3
#define DELAY_US_TIMER_PRESCALER        (SystemCoreClock / 1e6)


void enable_laser(void);
void disable_laser(void);
void set_apd_voltage(float new_voltage);
void init_all_hardware(void);
void delay_ms(uint32_t ms);
float calculate_real_temperature(uint16_t raw_value);
void auto_switch_apd_voltage(uint16_t current_amplitude);

#endif
