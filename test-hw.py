import os
import sys
import time
from smbus2 import SMBus
from imusensor.MPU9250 import MPU9250
from bmp280 import BMP280
from pymavlink import mavutil
import datetime

# MS4525DO Pressure Sensor I2C address and bus
MS4525DO_I2C_ADDR = 0x28
pressure_bus = SMBus(1)

# MPU9250 IMU Sensor setup
imu_address = 0x68
imu_bus = SMBus(1)
imu = MPU9250.MPU9250(imu_bus, imu_address)
imu.begin()

# BMP280 Sensor setup
bmp280_bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bmp280_bus)

# MAVLink connection setup
connection_string = '/dev/ttyAMA0'
baud_rate = 115200
connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Configure MAVLink message stream
message = connection.mav.command_long_encode(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    26,  # Message ID for Scaled_IMU
    100000,  # Interval in microseconds
    0, 0, 0, 0, 0
)
connection.mav.send(message)
response = connection.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("MAVLink Command accepted")
else:
    print("MAVLink Command failed")

def read_pressure_temperature():
    try:
        data = pressure_bus.read_i2c_block_data(MS4525DO_I2C_ADDR, 0, 4)
        pressure_raw = ((data[0] & 0x3F) << 8) | data[1]
        pressure = (pressure_raw - 8192) / 16384.0
        return pressure
    except Exception as e:
        print(f"Error reading from MS4525DO: {e}")
        return None

def print_sensor_data():
    while True:
        # Get current timestamp
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Read and display data from MS4525DO Pressure Sensor
        pressure = read_pressure_temperature()
        print(f"\n[{timestamp}] - MS4525DO Pressure Sensor Data")
        print(f"Pressure: {pressure:.4f}" if pressure is not None else "Pressure: Error reading data")

        # Read and display data from MPU9250 IMU Sensor
        imu.readSensor()
        imu.computeOrientation()
        print(f"\n[{timestamp}] - MPU9250 IMU Sensor Data")
        print(f"Accel x: {imu.AccelVals[0]} ; Accel y : {imu.AccelVals[1]} ; Accel z : {imu.AccelVals[2]}")
        print(f"Gyro x: {imu.GyroVals[0]} ; Gyro y : {imu.GyroVals[1]} ; Gyro z : {imu.GyroVals[2]}")
        print(f"Mag x: {imu.MagVals[0]} ; Mag y : {imu.MagVals[1]} ; Mag z : {imu.MagVals[2]}")
        print(f"roll: {imu.roll} ; pitch : {imu.pitch} ; yaw : {imu.yaw}")

        # Read and display data from MAVLink connection
        mavlink_data = connection.recv_match()
        print(f"\n[{timestamp}] - MAVLink Data")
        print(mavlink_data.to_dict() if mavlink_data else "No MAVLink data received")

        # Read and display data from BMP280 Sensor
        temperature = bmp280.get_temperature()
        pressure_bmp = bmp280.get_pressure()
        degree_sign = u"\N{DEGREE SIGN}"
        print(f"\n[{timestamp}] - BMP280 Sensor Data")
        print(f"Temperature: {temperature:.2f}{degree_sign}C")
        print(f"Pressure: {pressure_bmp:.2f} hPa")

        # Sleep for a short interval
        time.sleep(4)

if __name__ == "__main__":
    print_sensor_data()

