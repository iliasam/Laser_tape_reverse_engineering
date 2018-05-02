Firmware for "x-40" laser tape
By ILIASAM

This program captures data from ADC, calculate phase difference using Goertzel algorithm.
All this done sequentially for 3 frequencies. 
After this program calculate distance in mm, send results to UART.
Simultaneous data capture and data processing is used.

UART data example:
DIST;01574;AMP;0993;TEMP;1343;VOLT;080\r\n
DIST - distance in mm,  
AMP - received signal amplitude  
TEMP - raw temperature  
VOLT - APD voltage

String length is constant.
UART baudrate - 256000.
