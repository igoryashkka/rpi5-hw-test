import threading
import time
import subprocess
import random
from pymavlink import mavutil
import serial
import datetime
from colorama import Fore, Style, init
import numpy as np

# Initialize colorama for colored output
init(autoreset=True)

# UART Initialization
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
def read_one_mavlink_message(connection):
    try:
        message = connection.recv_match(blocking=False)
        if message:
            print(Fore.CYAN + f"MAVLink Message: {message.to_dict()}")
    except Exception as e:
        print(Fore.RED + f"Error receiving MAVLink message: {e}")

# Video Recording
def video_recording_loop():
    while True:
        try:
            print(Fore.CYAN + "Starting video recording...")
            subprocess.run(["libcamera-vid", "-o", "video_output.h264", "-t", "0"], check=True)
        except Exception as e:
            print(Fore.RED + f"Error during video recording: {e}")

# Sequential Sensor and UART Operations
def main_loop():
    i = 0
    try:
        mav_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
        mav_connection.wait_heartbeat()
        print(Fore.GREEN + "Heartbeat received from MAVLink system.")

        while True:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Read one MAVLink message
            read_one_mavlink_message(mav_connection)

            # NEO-M6 GPS
            if gps_serial:
                try:
                    if gps_serial.in_waiting > 0:
                        gps_data = gps_serial.readline().decode('ascii', errors='replace').strip()
                        print(Fore.YELLOW + f"NEO-M6 GPS: {gps_data}")
                except Exception as e:
                    print(Fore.RED + f"Error reading GPS data: {e}")

            # Simulate GPS Data for NMEA
            lat = 37.7925 + (i * 0.0001)
            lon = 30.4817 + (i * 0.0001)
            altitude = 100.0
            heading = random.uniform(0, 360)
            speed = random.uniform(0, 20)
            send_nmea(lat, lon, altitude, heading, speed)

            # Increment loop counter
            i = (i + 1) % 10
            time.sleep(1)
    except Exception as e:
        print(Fore.RED + f"Error in main loop: {e}")

if __name__ == "__main__":
    try:
        # Start video recording in a separate thread
        threading.Thread(target=video_recording_loop, daemon=True).start()

        # Run sequential operations in the main thread
        main_loop()
    except KeyboardInterrupt:
        print(Fore.RED + "Exiting program...")
