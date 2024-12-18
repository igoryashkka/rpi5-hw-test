"""
Example: Set that a message is streamed at particular rate
"""

from pymavlink import mavutil
import time
import serial
import numpy as np
from datetime import datetime
import random
ser = serial.Serial(
        port='/dev/ttyAMA3', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 115200)


#convert latitude or longatude in decimal form to form used by NMEA
def decimal_to_ddmm(decimal_degrees):   #d is degrees, m is minutes, format is ddmm.mmmm
    abs_latitude = abs(decimal_degrees)
    degrees = int(abs_latitude)
    minutes = (abs_latitude - degrees) * 60
    ddmm = (degrees * 100 + minutes)
    return ddmm



current_time_us = int(time.time() * 1e6)

def calculate_checksum(s):
    # Initialize the checksum to 0
    checksum = 0

    # XOR the ASCII values of all characters in the substring
    for char in s:
        checksum ^= ord(char)
    return hex(checksum)[2:]

def send_gps_data(lat, lon,absolute_height, clockwise_direction_heading,speed_meps):
    
    '''Geoidal separation, the difference between the WGS-84 earth
    ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid'''
    geoidal_separation = 0
    
    datastring = f'GPGGA,{datetime.utcnow().strftime("%H%M%S.%f")[:-3]},{abs(decimal_to_ddmm(lat))},{"N" if np.sign(lat)>0 else "S"},{abs(decimal_to_ddmm(lon))},{"E" if np.sign(lon)>0 else "W"},1,20,1.02,{absolute_height},M,{geoidal_separation},M,,'
    finalString = (f'${datastring}*{calculate_checksum(datastring)}\r\n')
    ser.write(finalString.encode())
    
    datastring = f'GPRMC,{datetime.utcnow().strftime("%H%M%S.%f")[:-3]},A,{abs(decimal_to_ddmm(lat))},{"N" if np.sign(lat)>0 else "S"},{abs(decimal_to_ddmm(lon))},{"E" if np.sign(lon)>0 else "W"},{speed_meps*4.63/9},{clockwise_direction_heading},{datetime.utcnow().strftime("%d%m%y")},,,A'
    finalString = (f'${datastring}*{calculate_checksum(datastring)}\r\n')
    ser.write(finalString.encode())

#Print gps data
i = 0
while True:
    time.sleep(.2)
    #send_gps_data(50.39251723326593,30.48178331772533,100,datetime.utcnow().strftime("%H%M%S.%f")[:-3],21)
    if i > 9:
        i = 0
    
    '''s = f'GPGGA,{datetime.utcnow().strftime("%H%M%S.%f")[:-3]}.00,532{i}.6802,N,00630.3371,W,1,8,1.03,61.7,M,55.3,M,,'
    ser.write(f'${s}*{calculate_checksum(s)}\r\n'.encode())
    
    s = f'GPRMC,{datetime.utcnow().strftime("%H%M%S.%f")[:-3]}.00,A,532{i}.6802,N,00630.3371,W,0.06,31.66,280511,,,A'
    ser.write(f'${s}*{calculate_checksum(s)}\r\n'.encode())
'''
    send_gps_data(50.39251723326593, 30.48178331772533, 100,0.2,0)
    i+=1





