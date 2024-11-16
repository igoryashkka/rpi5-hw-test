import threading
import time
import subprocess
import random
from pymavlink import mavutil
import serial
import datetime
import smbus
from imusensor.MPU9250 import MPU9250
from bmp280 import BMP280
from colorama import Fore, Style, init
import numpy as np

# Initialize colorama for colored output
init(autoreset=True)

# UART initialization
try:
    mavlink_serial = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)
except Exception as e:
    print(Fore.RED + f"Error initializing MAVLink UART: {e}")
    mavlink_serial = None

try:
    nmea_serial = serial.Serial('/dev/ttyAMA2', baudrate=115200, timeout=1)
except Exception as e:
    print(Fore.RED + f"Error initializing NMEA UART: {e}")
    nmea_serial = None

try:
    gps_serial = serial.Serial('/dev/ttyAMA3', baudrate=9600, timeout=1)
except Exception as e:
    print(Fore.RED + f"Error initializing GPS UART: {e}")
    gps_serial = None

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

# MS4525DO initialization
MS4525DO_I2C_ADDR = 0x28
pressure_bus = smbus.SMBus(1)

def read_pressure_temperature():
    try:
        data = pressure_bus.read_i2c_block_data(MS4525DO_I2C_ADDR, 0, 4)
        pressure_raw = ((data[0] & 0x3F) << 8) | data[1]
        pressure = (pressure_raw - 8192) / 16384.0
        return pressure
    except Exception as e:
        print(Fore.RED + f"Error reading from MS4525DO: {e}")
        return None

# NMEA Sentence Helpers
def decimal_to_ddmm(decimal_degrees):
    abs_val = abs(decimal_degrees)
    degrees = int(abs_val)
    minutes = (abs_val - degrees) * 60
    return degrees * 100 + minutes

def calculate_checksum(s):
    checksum = 0
    for char in s:
        checksum ^= ord(char)
    return hex(checksum)[2:].upper()

def send_nmea(lat, lon, altitude, heading, speed):
    if not nmea_serial:
        print(Fore.RED + "NMEA UART not initialized; skipping NMEA data send.")
        return

    try:
        gpgga_data = f'GPGGA,{datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-3]},{abs(decimal_to_ddmm(lat)):.4f},{"N" if lat >= 0 else "S"},{abs(decimal_to_ddmm(lon)):.4f},{"E" if lon >= 0 else "W"},1,10,0.95,{altitude},M,0,M,,'
        gpgga_sentence = f'${gpgga_data}*{calculate_checksum(gpgga_data)}\r\n'
        nmea_serial.write(gpgga_sentence.encode())
        print(Fore.BLUE + f"Sent: {gpgga_sentence.strip()}")

        gprmc_data = f'GPRMC,{datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-3]},A,{abs(decimal_to_ddmm(lat)):.4f},{"N" if lat >= 0 else "S"},{abs(decimal_to_ddmm(lon)):.4f},{"E" if lon >= 0 else "W"},{speed * 1.94384:.2f},{heading:.2f},{datetime.datetime.utcnow().strftime("%d%m%y")},,,A'
        gprmc_sentence = f'${gprmc_data}*{calculate_checksum(gprmc_data)}\r\n'
        nmea_serial.write(gprmc_sentence.encode())
        print(Fore.BLUE + f"Sent: {gprmc_sentence.strip()}")

    except Exception as e:
        print(Fore.RED + f"Error sending NMEA data: {e}")

# MAVLink Reader
def read_mavlink():
    if not mavlink_serial:
        print(Fore.RED + "MAVLink UART not initialized; skipping MAVLink reading.")
        return

    mav_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    while True:
        try:
            message = mav_connection.recv_match(blocking=False)
            if message:
                print(Fore.CYAN + f"MAVLink: {message.to_dict()}")
        except Exception as e:
            print(Fore.RED + f"Error reading MAVLink data: {e}")

# GPS Reader
def read_gps():
    if not gps_serial:
        print(Fore.RED + "GPS UART not initialized; skipping GPS reading.")
        return

    while True:
        try:
            if gps_serial.in_waiting > 0:
                gps_data = gps_serial.readline().decode('ascii', errors='replace').strip()
                print(Fore.YELLOW + f"NEO-M6 GPS: {gps_data}")
        except Exception as e:
            print(Fore.RED + f"Error reading GPS data: {e}")

# IMU and BMP280 Reader
def read_sensors():
    while True:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # MPU9250
        if imu:
            imu.readSensor()
            imu.computeOrientation()
            print(Fore.YELLOW + f"\n[{timestamp}] - IMU Data")
            print(f"Accel x: {imu.AccelVals[0]} ; Accel y: {imu.AccelVals[1]} ; Accel z: {imu.AccelVals[2]}")
            print(f"Gyro x: {imu.GyroVals[0]} ; Gyro y: {imu.GyroVals[1]} ; Gyro z: {imu.GyroVals[2]}")
            print(f"Roll: {imu.roll:.2f} ; Pitch: {imu.pitch:.2f} ; Yaw: {imu.yaw:.2f}")

        # BMP280
        if bmp280:
            temperature = bmp280.get_temperature()
            pressure_bmp = bmp280.get_pressure()
            print(Fore.GREEN + f"Temperature: {temperature:.2f}°C, Pressure: {pressure_bmp:.2f} hPa")

        # MS4525DO
        pressure = read_pressure_temperature()
        print(Fore.MAGENTA + f"Pitot Tube Pressure: {pressure if pressure else 'Error reading data'}")

        time.sleep(1)

# Video Recording
def video_recording_loop():
    while True:
        try:
            print(Fore.CYAN + "Starting video recording...")
            subprocess.run(["libcamera-vid", "-o", "video_output.h264", "-t", "0"], check=True)
        except Exception as e:
            print(Fore.RED + f"Error during video recording: {e}")

# Simulate GPS Data for NMEA
def simulate_gps():
    i = 0
    while True:
        lat = 37.7925 + (i * 0.0001)
        lon = 30.4817 + (i * 0.0001)
        altitude = 100.0
        heading = random.uniform(0, 360)
        speed = random.uniform(0, 20)
        send_nmea(lat, lon, altitude, heading, speed)
        i = (i + 1) % 10
        time.sleep(1)

if __name__ == "__main__":
    try:
        # Threads for parallel tasks
        threading.Thread(target=read_mavlink, daemon=True).start()
        threading.Thread(target=read_gps, daemon=True).start()
        threading.Thread(target=read_sensors, daemon=True).start()
        threading.Thread(target=video_recording_loop, daemon=True).start()
        threading.Thread(target=simulate_gps, daemon=True).start()

        # Keep main thread alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print(Fore.RED + "Exiting program...")
