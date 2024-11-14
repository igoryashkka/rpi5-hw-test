"""
Example: Set that a message is streamed at particular rate
"""

from pymavlink import mavutil
import time
import threading
#from apscheduler.schedulers.background import BackgroundScheduler
import numpy as np

# Replace '/dev/ttyACM0' with the correct device name if different
connection_string = '/dev/ttyAMA0'
baud_rate = 115200

# Connect to the flight controller via USB
connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
# param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
# param2: 1000000 (Stream interval in microseconds)
message = connection.mav.command_long_encode(
        connection.target_system,  # Target system ID
        connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        26,  # param1: Message ID to be streamed, 26 - Scaled_IMU
        100000, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )

# Send the COMMAND_LONG
connection.mav.send(message)

# Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
response = connection.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Command accepted")
else:
    print("Command failed")

while True:
    time.sleep(0.01)
    try:
        print(connection.recv_match().to_dict())
    except:
        pass

