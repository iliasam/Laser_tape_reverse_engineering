All this firmware is written for the STM32 MCU of the rangefinder module.  
Used IDE - IAR 7.50.  
See programming instructions: https://github.com/iliasam/Laser_tape_reverse_engineering/wiki/U85-module-programming  
  
"_boot.hex" is compiled firmware for using  with bootloader, if user do not need SWD debugging.  
SWD interface is disabled after 1s after firmware startup - to free UART TX pin.  
So UART would be working in any case. Firmware could be changed with DFU UART bootloader OR with programmer (use NRST!).  
  
Not "_boot.hex" firmware is check state of MCU UART RX pin at startup.  
If this pin (RX) is not pulled up from external UART, then MCU UART is disabled, so SWD could be used for debugging.  
"DETECT_UART_PRESENCE" define is active at this mode.

"Firmware_dist_calculation_fast":
This program captures data from ADC, calculate phase difference, 
calculate distance in mm, send results to the UART.

**************************************************************************************************

"Firmware_dist_calculation_simple":
This program captures data from ADC, calculate phase difference using Goertzel algorithm, 
calculate distance in mm, send results to the UART.
This program uses plain data capture algorithm, so it's not fast.

**************************************************************************************************

"Firmware_raw_capture":
Program captures data from ADC and send it to the UART.

**************************************************************************************************

"Firmware_phase_calculation":
This program captures data from ADC, calculate phase difference using Goertzel algorithm, send results to UART.  
"test_firmware_phase_calc.hex" - same as previous, but phase compensation of amplitude changing is disabled.  UART is always active.  

See also PC utilities desctription: https://github.com/iliasam/Laser_tape_reverse_engineering/tree/master/PC_utility
