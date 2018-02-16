# laser_tape_reverse_engineering 

This project describes my work about reverse engineering electronics of cheap "X-40" laser tape.

Article in Russan: https://habrahabr.ru/post/327642

Video: https://youtu.be/bJaUrZ7ZMj4

Steps that I have done:  
- Fully reverse engineered schematic of laser tape.   
- Captured data packets at I2C bus with logic analyzer.  
- Decoded that packets and get values of laser modulation frequencies.  
- Write own firmware for that captures low frequency signal and send it to PC.  
- Write own firmware for that captures signal and processing if using Goertzel algorithm. Phase difference results are send to PC.  
- Main result: Write own firmware ('Firmware_dist_calculation_fast") that calculates distance to object.  
- Write C# PC utilities to process and show results.

Main parts of laser tape are STM32F100C8T6 MCU, Si5351 dual PLL, APD (unknown type).  
Laser tape structure schematic:  
![Alt text](Schematic/schematic_structure.png?raw=true "Image")  
  
Loading firmware to laser tape's MCU.  
Connect pins 7 and 8 at keyboard connector (or constantly press "S1" key). That needed to enable laser tape DC-DC converter.  
You need any STM32 programmer that has NRST pin.  
Connect SWCLK, SWDIO, NRST, GND from laser tape to programmer.  
![Alt text](PCB_photos/PCB_top.JPG?raw=true "Image")
Configure programmer to use "Connect under reset".  
Power on laser tape.  
Erase whole STM32 FLASH memory. WARNING: YOU WILL LOSE ORIGINAL LASER TAPE FIRMWARE AT THIS STEP! You are doing it at your own risk!  
Write needed firmware to SM32.  


UART data example ("Firmware_dist_calculation_fast"):  
DIST;01574;AMP;0993;TEMP;1343;VOLT;082\r\n  
DIST - distance to object in mm.  
AMP  - signal amplitude. 
TEMP - APD temperature (raw ADC value).  
VOLT - APD voltage. 
String length is constant.  
UART baudrate - 256000.  

Distance measurement speed - near 60 Hz.  

UART commands ("Firmware_dist_calculation_fast"):  
"E" - enable laser and measurement process.  
"D" - disable laser and measurement process.  
"C" - start zero distance calibration. You need to run zero distance calibration at the first start. Place any white object at the distance > 10cm from the laser tape before calibration.  


To get better results, small board with APD must be closed from external light.

