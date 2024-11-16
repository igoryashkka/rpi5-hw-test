import threading
import time
import random
from pymavlink import mavutil
import serial
import datetime
import numpy as np
from colorama import Fore, Style, init

# Initialize colorama for colored output
init(autoreset=True)

# Initialize UARTs
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


# Send NMEA sentences
def send_nmea(lat, lon, altitude, heading, speed):
    if not nmea_serial:
        print(Fore.RED + "NMEA UART not initialized; skipping NMEA data send.")
        return

    try:
        # GPGGA sentence
        geoidal_separation = 0
        gpgga_data = f'GPGGA,{datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-3]},{abs(decimal_to_ddmm(lat)):.4f},{"N" if lat >= 0 else "S"},{abs(decimal_to_ddmm(lon)):.4f},{"E" if lon >= 0 else "W"},1,10,0.95,{altitude},M,{geoidal_separation},M,,'
        gpgga_sentence = f'${gpgga_data}*{calculate_checksum(gpgga_data)}\r\n'
        nmea_serial.write(gpgga_sentence.encode())
        print(Fore.BLUE + f"Sent: {gpgga_sentence.strip()}")

        time.sleep(0.1)

        # GPRMC sentence
        gprmc_data = f'GPRMC,{datetime.datetime.utcnow().strftime("%H%M%S.%f")[:-3]},A,{abs(decimal_to_ddmm(lat)):.4f},{"N" if lat >= 0 else "S"},{abs(decimal_to_ddmm(lon)):.4f},{"E" if lon >= 0 else "W"},{speed * 1.94384:.2f},{heading:.2f},{datetime.datetime.utcnow().strftime("%d%m%y")},,,A'
        gprmc_sentence = f'${gprmc_data}*{calculate_checksum(gprmc_data)}\r\n'
        nmea_serial.write(gprmc_sentence.encode())
        print(Fore.BLUE + f"Sent: {gprmc_sentence.strip()}")

    except Exception as e:
        print(Fore.RED + f"Error sending NMEA data: {e}")


# Read MAVLink data
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


# Read NEO-M6 GPS data
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


# Main loop to simulate GPS and send NMEA sentences
def simulate_gps():
    i = 0
    while True:
        try:
            # Generate simulated data
            lat = 37.7925 + (i * 0.0001)
            lon = 30.4817 + (i * 0.0001)
            altitude = 100.0
            heading = random.uniform(0, 360)
            speed = random.uniform(0, 20)

            # Send simulated NMEA data
            send_nmea(lat, lon, altitude, heading, speed)

            # Increment loop counter
            i = (i + 1) % 10
            time.sleep(1)
        except Exception as e:
            print(Fore.RED + f"Error in GPS simulation: {e}")


if __name__ == "__main__":
    try:
        # Start threads for MAVLink, GPS, and NMEA
        mavlink_thread = threading.Thread(target=read_mavlink, daemon=True)
        gps_thread = threading.Thread(target=read_gps, daemon=True)
        simulate_thread = threading.Thread(target=simulate_gps, daemon=True)

        mavlink_thread.start()
        gps_thread.start()
        simulate_thread.start()

        # Keep main thread alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print(Fore.RED + "Exiting program...")
