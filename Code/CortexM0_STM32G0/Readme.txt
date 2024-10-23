All this firmware is written for the STM32 MCU of the rangefinder module.  
Used IDE - Keil 5.24.  
  
"Firmware_dist_calculation_simple":
This program captures data from ADC, calculate phase difference using Goertzel algorithm, 
calculate distance in mm, send results to the UART.
This program uses plain data capture algorithm, so it's not fast. Measurement speed is near 30Hz.  

**************************************************************************************************

"Firmware_raw_capture":
Program captures data from ADC and send it to the UART.

**************************************************************************************************

"Firmware_phase_calculation":
This program captures data from ADC, calculate phase difference using Goertzel algorithm, send results to UART.  
"test_firmware_phase_calc.hex" - same as previous, but phase compensation of amplitude changing is disabled.  UART is always active.  

See also PC utilities desctription: https://github.com/iliasam/Laser_tape_reverse_engineering/tree/master/PC_utility
