import math
import os
import re

prev_start_time = 0
data_line_cnt = 0
big_packet_cnt = 0

data_array = []
old_data_array = []
register_array = []

#process new line
def process_new_line(cur_str): 
    global prev_start_time
    global data_line_cnt
    global data_array
    global old_data_array
    if (cur_str.find('Write')>0): #begin of new packet
        analyse_data(data_line_cnt, prev_start_time)
        print ('This packet size: ' + str(data_line_cnt)) #will be printed at the end of prev packet
        print (' ') #end of packet
        data_line_cnt = 0
        data_array = []
        
        print (cur_str.rstrip())
        cur_start_time = float(re.findall(r'[+-]?[0-9.]+', cur_str)[0])
        delta_time = cur_start_time - prev_start_time
        prev_start_time =  cur_start_time
        print ('Time from last packet: ' + str(round(delta_time * 1000, 3)) + " ms")
        
    else: #data line
        data_line_cnt+=1
        data_byte = int(re.findall(r'0[xX][0-9a-fA-F]+', cur_str)[0], 16)
        data_array.append(data_byte)

def analyse_data(length, start_time):
    global file_wr
    global data_array
    dat_list = ';'.join(map(str, data_array))
    result = str(start_time) + ';' + str(length) + ';' + dat_list
    print (result)
    file_wr.write(result + '\n')


#**********************************************************

os.chdir('..')
file_an = open('../init_and_2_measureA.csv', 'r')
#file_an.readline() #header
cur_str = file_an.readline()

file_wr = open('results.csv', 'w')
file_wr.write('TIME;LENGTH;DATA' + '\r\n') #header

while (cur_str != ''): #while eof not found
    cur_str = file_an.readline() #read new line
    if (cur_str != ''):
        process_new_line(cur_str)
file_wr.close()
 

