Firmware for "x-40" laser tape
By ILIASAM

This program captures data from ADC, calculate phase difference using Goertzel algorithm.
All this done sequentially for 3 frequencies. 
After this program calculate distance in mm, send results to UART.
This program uses plain data capture algorithm, so it's not fast.

UART data example:
DIST;01574;AMP;0993;TEMP;1343;VOLT;080\r\n
String length is constant.
UART baudrate - 256000.
