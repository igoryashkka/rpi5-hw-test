import os
import sys
import time
import subprocess
import threading
import smbus
from imusensor.MPU9250 import MPU9250
from bmp280 import BMP280
from pymavlink import mavutil
import datetime
import serial
from colorama import Fore, Style, init

# Initialize colorama
init(autoreset=True)

# MS4525DO Pressure Sensor I2C address and bus
MS4525DO_I2C_ADDR = 0x28
pressure_bus = smbus.SMBus(1)

# Initialize sensors with error handling
imu, bmp280, ser, connection = None, None, None, None

try:
    # MPU9250 IMU Sensor setup
    imu_address = 0x68
    imu_bus = smbus.SMBus(1)
    imu = MPU9250.MPU9250(imu_bus, imu_address)
    imu.begin()
except Exception as e:
    print(Fore.RED + f"Error initializing MPU9250 IMU: {e}")

try:
    # BMP280 Sensor setup
    bmp280_bus = smbus.SMBus(1)
    bmp280 = BMP280(i2c_dev=bmp280_bus)
except Exception as e:
    print(Fore.RED + f"Error initializing BMP280: {e}")

try:
    # Serial setup for communication
    ser = serial.Serial('/dev/ttyAMA10', 9600, timeout=1)
except Exception as e:
    print(Fore.RED + f"Error initializing serial communication: {e}")

try:
    # MAVLink connection setup
    connection_string = '/dev/ttyAMA0'
    baud_rate = 115200
    connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)
    connection.wait_heartbeat(timeout=10)
    print(Fore.GREEN + "Heartbeat received from MAVLink system")
except Exception as e:
    print(Fore.RED + f"MAVLink connection failed: {e}")
    connection = None

def read_pressure_temperature():
    """Read data from MS4525DO Pressure Sensor."""
    try:
        data = pressure_bus.read_i2c_block_data(MS4525DO_I2C_ADDR, 0, 4)
        pressure_raw = ((data[0] & 0x3F) << 8) | data[1]
        pressure = (pressure_raw - 8192) / 16384.0
        return pressure
    except Exception as e:
        print(Fore.RED + f"Error reading from MS4525DO: {e}")
        return None

def video_recording_loop(filename="video.h264"):
    """Continuous video recording from Raspberry Pi camera."""
    while True:
        try:
            print(Fore.CYAN + f"Starting video recording: {filename}")
            subprocess.run([
                "libcamera-vid",
                "-o", filename,
                "-t", "0"  # Continuous recording
            ], check=True)
        except Exception as e:
            print(Fore.RED + f"Error during video recording: {e}")
            break

def sensor_data_loop():
    """Read sensor data in a loop."""
    while True:
        # Get current timestamp
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Read and display serial data
        if ser and ser.in_waiting > 0:
            line = ser.readline().decode('ascii', errors='replace').strip()
            print(Fore.BLUE + f"\n[{timestamp}] - GPS Sensor Data")
            print(line)

        # Read and display data from MS4525DO Pressure Sensor
        pressure = read_pressure_temperature()
        print(Fore.MAGENTA + f"\n[{timestamp}] - MS4525DO Pressure Sensor Data")
        print(f"Pressure: {pressure:.4f}" if pressure is not None else "Pressure: Error reading data")

        # Read and display data from MPU9250 IMU Sensor
        if imu:
            imu.readSensor()
            imu.computeOrientation()
            print(Fore.YELLOW + f"\n[{timestamp}] - MPU9250 IMU Sensor Data")
            print(f"Accel x: {imu.AccelVals[0]} ; Accel y : {imu.AccelVals[1]} ; Accel z : {imu.AccelVals[2]}")
            print(f"Gyro x: {imu.GyroVals[0]} ; Gyro y : {imu.GyroVals[1]} ; Gyro z : {imu.GyroVals[2]}")
            print(f"Mag x: {imu.MagVals[0]} ; Mag y : {imu.MagVals[1]} ; Mag z : {imu.MagVals[2]}")
            print(f"roll: {imu.roll} ; pitch : {imu.pitch} ; yaw : {imu.yaw}")
        else:
            print(Fore.RED + "MPU9250 IMU Sensor not initialized")

        # Read and display data from MAVLink connection if connected
        if connection:
            mavlink_data = connection.recv_match()
            print(Fore.CYAN + f"\n[{timestamp}] - MAVLink Data")
            print(mavlink_data.to_dict() if mavlink_data else "No MAVLink data received")
        else:
            print(Fore.RED + f"\n[{timestamp}] - MAVLink Data not available")

        # Read and display data from BMP280 Sensor
        if bmp280:
            temperature = bmp280.get_temperature()
            pressure_bmp = bmp280.get_pressure()
            degree_sign = u"\N{DEGREE SIGN}"
            print(Fore.GREEN + f"\n[{timestamp}] - BMP280 Sensor Data")
            print(f"Temperature: {temperature:.2f}{degree_sign}C")
            print(f"Pressure: {pressure_bmp:.2f} hPa")
        else:
            print(Fore.RED + "BMP280 Sensor not initialized")

        # Sleep for a short interval
        time.sleep(4)

if __name__ == "__main__":
    try:
        # Start video recording in a separate thread
        video_thread = threading.Thread(target=video_recording_loop, args=("video_output.h264",), daemon=True)
        video_thread.start()

        # Start sensor data reading loop in the main thread
        sensor_data_loop()
    except KeyboardInterrupt:
        print(Fore.RED + "Exiting program...")
