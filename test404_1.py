import os
import time
import subprocess
import threading
import smbus
from imusensor.MPU9250 import MPU9250
from bmp280 import BMP280
from pymavlink import mavutil
import datetime
import serial
import numpy as np
from colorama import Fore, Style, init
import random

# Initialize colorama for colored output
init(autoreset=True)

# UART setup
try:
    ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200)
except Exception as e:
    print(Fore.RED + f"Error initializing UART: {e}")
    ser = None

# Convert latitude or longitude to NMEA format (ddmm.mmmm)
def decimal_to_ddmm(decimal_degrees):
    abs_val = abs(decimal_degrees)
    degrees = int(abs_val)
    minutes = (abs_val - degrees) * 60
    return degrees * 100 + minutes

# Calculate NMEA checksum
def calculate_checksum(s):
    checksum = 0
    for char in s:
        checksum ^= ord(char)
    return hex(checksum)[2:].upper()

# Send GPS data (NMEA sentences)
def send_gps_data(lat, lon, altitude, heading, speed):
    if not ser:
        print(Fore.RED + "UART not initialized; skipping GPS data send.")
        return
    
    try:
        # GPGGA sentence
        geoidal_separation = 0  # Example geoidal separation
        gpgga_data = f'GPGGA,{datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-3]},{abs(decimal_to_ddmm(lat)):.4f},{"N" if lat >= 0 else "S"},{abs(decimal_to_ddmm(lon)):.4f},{"E" if lon >= 0 else "W"},1,10,0.95,{altitude},M,{geoidal_separation},M,,'
        gpgga_sentence = f'${gpgga_data}*{calculate_checksum(gpgga_data)}\r\n'
        ser.write(gpgga_sentence.encode())
        print(Fore.BLUE + f"Sent: {gpgga_sentence.strip()}")

        time.sleep(0.1)

        # GPRMC sentence
        gprmc_data = f'GPRMC,{datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-3]},A,{abs(decimal_to_ddmm(lat)):.4f},{"N" if lat >= 0 else "S"},{abs(decimal_to_ddmm(lon)):.4f},{"E" if lon >= 0 else "W"},{speed * 1.94384:.2f},{heading:.2f},{datetime.datetime.utcnow().strftime("%d%m%y")},,,A'
        gprmc_sentence = f'${gprmc_data}*{calculate_checksum(gprmc_data)}\r\n'
        ser.write(gprmc_sentence.encode())
        print(Fore.BLUE + f"Sent: {gprmc_sentence.strip()}")

        time.sleep(0.1)

    except Exception as e:
        print(Fore.RED + f"Error sending GPS data: {e}")

# Sensor initialization
try:
    imu_bus = smbus.SMBus(1)
    imu = MPU9250.MPU9250(imu_bus, 0x68)
    imu.begin()
except Exception as e:
    print(Fore.RED + f"Error initializing MPU9250: {e}")
    imu = None

try:
    bmp280 = BMP280(i2c_dev=smbus.SMBus(1))
except Exception as e:
    print(Fore.RED + f"Error initializing BMP280: {e}")
    bmp280 = None

# Video recording loop
def video_recording_loop(filename="video_output.h264"):
    while True:
        try:
            print(Fore.CYAN + f"Starting video recording: {filename}")
            subprocess.run(["libcamera-vid", "-o", filename, "-t", "0"], check=True)
        except Exception as e:
            print(Fore.RED + f"Error during video recording: {e}")
            break

# Main sensor loop
def sensor_data_loop():
    i = 0
    while True:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # MPU9250 Data
        if imu:
            imu.readSensor()
            imu.computeOrientation()
            print(Fore.YELLOW + f"\n[{timestamp}] - IMU Sensor Data")
            print(f"Accel x: {imu.AccelVals[0]} ; Accel y : {imu.AccelVals[1]} ; Accel z : {imu.AccelVals[2]}")
            print(f"Gyro x: {imu.GyroVals[0]} ; Gyro y : {imu.GyroVals[1]} ; Gyro z : {imu.GyroVals[2]}")
            print(f"roll: {imu.roll} ; pitch : {imu.pitch} ; yaw : {imu.yaw}")

        # BMP280 Data
        if bmp280:
            temperature = bmp280.get_temperature()
            pressure_bmp = bmp280.get_pressure()
            print(Fore.GREEN + f"\n[{timestamp}] - BMP280 Sensor Data")
            print(f"Temperature: {temperature:.2f}°C ; Pressure: {pressure_bmp:.2f} hPa")

        # Simulated GPS Data
        lat = 37.7925 + (i * 0.0001)
        lon = 30.4817 + (i * 0.0001)
        altitude = 100.0
        heading = random.uniform(0, 360)
        speed = random.uniform(0, 20)
        send_gps_data(lat, lon, altitude, heading, speed)
        i = (i + 1) % 10

        time.sleep(1)

if __name__ == "__main__":
    try:
        # Start video recording in a separate thread
        video_thread = threading.Thread(target=video_recording_loop, daemon=True)
        video_thread.start()

        # Start main sensor loop
        sensor_data_loop()
    except KeyboardInterrupt:
        print(Fore.RED + "Exiting program...")
