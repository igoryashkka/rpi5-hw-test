import smbus2
import time

# Define the I2C address of the IST8308 magnetometer
IST8308_ADDRESS = 0x1E

# Define IST8308 registers
IST8308_REG_WHO_AM_I = 0x00  # Who Am I register (for device identification)
IST8308_REG_DATA_START = 0x03  # Data output starting register
IST8308_REG_CTRL = 0x0A       # Control register

# Sensitivity of the IST8308 magnetometer (example: 0.3 μT per count, check the datasheet)
SENSITIVITY = 0.3  # μT per count

# Initialize the I2C bus
bus = smbus2.SMBus(1)  # I2C bus number (1 is typical for Raspberry Pi)

def initialize_ist8308():
    """Initializes the IST8308 magnetometer."""
    # Verify device identity (optional)
    who_am_i = bus.read_byte_data(IST8308_ADDRESS, IST8308_REG_WHO_AM_I)
    if who_am_i != 0x10:  # Replace 0x10 with the expected ID for IST8308
        raise RuntimeError(f"Unexpected WHO_AM_I response: {who_am_i:#04x}")
    print("IST8308 detected.")

    # Set control register to start continuous measurement mode
    bus.write_byte_data(IST8308_ADDRESS, IST8308_REG_CTRL, 0x01)


def read_magnetometer():
    """Reads raw magnetometer data from the IST8308 and converts to μT."""
    # Read 6 bytes of data starting at the data output register
    data = bus.read_i2c_block_data(IST8308_ADDRESS, IST8308_REG_DATA_START, 6)

    # Combine high and low bytes for X, Y, and Z axes
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    # Convert to signed 16-bit values
    if x >= 0x8000:
        x -= 0x10000
    if y >= 0x8000:
        y -= 0x10000
    if z >= 0x8000:
        z -= 0x10000

    # Convert to microteslas
    x_uT = x * SENSITIVITY
    y_uT = y * SENSITIVITY
    z_uT = z * SENSITIVITY

    return x_uT, y_uT, z_uT

# Main program
try:
    initialize_ist8308()

    while True:
        x_uT, y_uT, z_uT = read_magnetometer()
        print(f"Magnetometer readings: X={x_uT:.2f} μT, Y={y_uT:.2f} μT, Z={z_uT:.2f} μT")
        time.sleep(0.5)  # Adjust delay as needed

except KeyboardInterrupt:
    print("Program terminated.")
except Exception as e:
    print(f"Error: {e}")
finally:
    bus.close()

