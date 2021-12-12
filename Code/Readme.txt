Important notice - all this firmware has been tested only with 512A and 701A hardware!
512A board revision is selected if MODULE_701A not defined is "main.h" file.  
All this firmware is written for the STM32 MCU of the rangefinder module.  
Used IDE - IAR 7.50.  

"CortexM0" - folder with fimware for "U85B" modules.  
  
"Firmware_dist_calculation_fast":
This program captures data from ADC, calculate phase difference using Goertzel algorithm, 
calculate distance in mm, send results to the UART.

**************************************************************************************************

"Firmware_dist_calculation_simple":
Only "512A" module is supported!!!!  
This program captures data from ADC, calculate phase difference using Goertzel algorithm, 
calculate distance in mm, send results to the UART.
This program uses plain data capture algorithm, so it's not fast.

**************************************************************************************************

"Firmware_raw_capture":
Program captures data from ADC and send it to the UART.

**************************************************************************************************

"Firmware_phase_calculation":
This program captures data from ADC, calculate phase difference using Goertzel algorithm, send results to UART.

See also PC utilities desctription: https://github.com/iliasam/Laser_tape_reverse_engineering/tree/master/PC_utility
