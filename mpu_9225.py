import os
import sys
import time
import smbus
from imusensor.MPU9250 import MPU9250


address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()


output_file = "imu_data.txt"


with open(output_file, "w") as file:

    file.write("Accel_x,Accel_y,Accel_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z,Roll,Pitch,Yaw\n")
    
    while True:
        try:
           
            imu.readSensor()
            imu.computeOrientation()
            
            
            accel_x, accel_y, accel_z = imu.AccelVals
            gyro_x, gyro_y, gyro_z = imu.GyroVals
            mag_x, mag_y, mag_z = imu.MagVals
            roll, pitch, yaw = imu.roll, imu.pitch, imu.yaw
            
            
            print(f"Accel x: {accel_x} ; Accel y : {accel_y} ; Accel z : {accel_z}")
            print(f"Gyro x: {gyro_x} ; Gyro y : {gyro_y} ; Gyro z : {gyro_z}")
            print(f"Mag x: {mag_x} ; Mag y : {mag_y} ; Mag z : {mag_z}")
            print(f"roll: {roll} ; pitch : {pitch} ; yaw : {yaw}")
            
        
            file.write(f"{accel_x},{accel_y},{accel_z},{gyro_x},{gyro_y},{gyro_z},"
                       f"{mag_x},{mag_y},{mag_z},{roll},{pitch},{yaw}\n")
            
            
            file.flush()
            
        
            time.sleep(0.1)
        
        except KeyboardInterrupt:
            
            print("Terminating...")
            break
