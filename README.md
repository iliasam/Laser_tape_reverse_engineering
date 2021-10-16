# laser_tape_reverse_engineering 

This project describes my work about reverse engineering electronics (laser rangefinder module) of a cheap "X-40" laser tape measure.  
I create my own firmware that allow to use this module for DIY purposes.  
Supported module types are: "512A" and "701A". Working of "703A" is confirmed by "Damian" here: [Link](https://www.hackster.io/iliasam/making-a-cheap-laser-rangefinder-for-arduino-4dd849).  
UPD 11/2019: Attention: It was found that newer "x-40" laser tape measures come with "M88B" modules. Marking is "88B" at the PCB near MCU. That modules are based on STM32F0 MCU (it has QFN-32 package). That modules are not supported by my firmware!  
Module dimensions: 25x13x50 mm.  
  
**UPD 10/2021: Added support for "U85B" modules that are used in modern lasr tape measures!**  
See: https://github.com/iliasam/Laser_tape_reverse_engineering/wiki/U85B-laser-rangefinder-modules  
  
Big article in Russian: https://habr.com/post/327642/  
Google translated: [Link](http://translate.google.com/translate?sl=ru&tl=en&js=y&prev=_t&hl=en&ie=UTF-8&u=https%3A%2F%2Fhabr.com%2Fpost%2F327642%2F&edit-text=).  
Another project page: https://hackaday.io/project/25515-cheap-laser-tape-measure-reverse-engineering  

Steps that I have done:  
- Fully reverse engineered schematic of the laser tape measure.   
- Captured data packets at I2C bus with logic analyzer.  
- Decoded that packets and get values of laser modulation frequencies.  
- Create my own firmware that captures low frequency signal and send it to PC.  
- Create my own firmware that captures signal and processing if using Goertzel algorithm. Phase difference results are send to the PC.  
- Main result: Create my own firmware ("Firmware_dist_calculation_fast") that calculates distance to the object.  
- Write C# PC utilities to process and show results.  

Video (testing rangefinder module): https://youtu.be/bJaUrZ7ZMj4  

Main parts of the laser rangefinder module are STM32F100C8T6 MCU, Si5351 dual PLL, APD (unknown type), laser diode, power sources.  
Laser tape measure structure schematic:  
![Alt text](Schematic/schematic_structure.png?raw=true "Image")  
  
Article about connecting laser rangefinder module to the Arduino:  
https://www.hackster.io/iliasam/making-a-cheap-laser-rangefinder-for-arduino-4dd849  
Video: https://youtu.be/FA4mfvgpOQQ  


UART data example ("Firmware_dist_calculation_fast"):  
DIST;01574;AMP;0993;TEMP;1343;VOLT;082\r\n  
DIST - distance to object in mm.  
AMP  - signal amplitude. 
TEMP - APD temperature (raw ADC value).  
VOLT - APD voltage. 
String length is constant.  
Distance data are transmitted continuously.  
UART baudrate - 256000.  

UART commands ("Firmware_dist_calculation_fast"):  
"E" - enable laser and measurement process.  
"D" - disable laser and measurement process.  
"C" - start zero distance calibration. You need to run zero distance calibration at the first start. Place any white object at the distance > 10cm from the laser tape before calibration.  

Distance measurement speed - near 60 Hz.  
Distance measurement accuracy vary from 1 to 10 mm depending from the distance and surface type.  

To get better results, small board with APD must be closed from external light.  
