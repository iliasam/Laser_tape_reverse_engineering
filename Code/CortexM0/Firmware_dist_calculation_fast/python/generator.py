import re
import csv
import math
from datetime import date

POINTS_TO_SAMPLE = 250
INT_COEF = 1024.0
SCALING = POINTS_TO_SAMPLE / 2.0
SIGNAL_FREQ = 5000.0
ADC_TRIGGER_FREQ_HZ = 50000.0
K_COEF = (SIGNAL_FREQ * POINTS_TO_SAMPLE / ADC_TRIGGER_FREQ_HZ)

OMEGA = ((2.0 * math.pi * K_COEF) / POINTS_TO_SAMPLE)


file_wr = open('analyse_table.h', 'w')

today = date.today()
file_wr.write('//Generated: ' + str(today) + '\r\n')
file_wr.write('\r\n')

file_wr.write('static const int32_t analyse_sin_table[] = {\r\n')
for x in range(POINTS_TO_SAMPLE):
    var = math.sin(x * OMEGA) * INT_COEF;
    if ((x > 0) & ((x % 10) == 0)): #just formating
        file_wr.write('\r\n')
    #file_wr.write('0x00,')
    file_wr.write(str(int(var)) + ',')
file_wr.write('};\r\n\r\n')

file_wr.write('static const int32_t analyse_cos_table[] = {\r\n')
for x in range(POINTS_TO_SAMPLE):
    var = math.cos(x * OMEGA) * INT_COEF;
    if ((x > 0) & ((x % 10) == 0)): #just formating
        file_wr.write('\r\n')
    #file_wr.write('0x00,')
    file_wr.write(str(int(var)) + ',')
file_wr.write('};\r\n\r\n')

file_wr.close()
exit()


