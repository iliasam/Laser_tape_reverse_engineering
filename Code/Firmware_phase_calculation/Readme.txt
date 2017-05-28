Firmware for "x-40" laser tape
By ILIASAM
This program captures data from ADC, calculate phase difference using Goertzel algorithm, send results to UART.
All this done sequentially for 3 frequencies.
UART data example:
> freqA_amp:352
> APD temp:1417
> Volt:95
> freqA_phase:3293
> freqB_phase:70
> freqC_phase:94
Phase units - 0.1 deg.
UART baudrate - 256000.
