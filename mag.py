import smbus2
import time


I2C_BUS = 1
MAGNETOMETER_ADDRESS = 0x1E


WHO_AM_I_REG = 0x00  
DATA_X_LSB = 0x03    
DATA_X_MSB = 0x04    
DATA_Y_LSB = 0x05    
DATA_Y_MSB = 0x06    
DATA_Z_LSB = 0x07    
DATA_Z_MSB = 0x08    
CTRL1 = 0x0A         
CTRL2 = 0x0B         


bus = smbus2.SMBus(I2C_BUS)

def initialize_ist8308():
    """Initialize the IST8308 magnetometer."""
    try:
        
        device_id = bus.read_byte_data(MAGNETOMETER_ADDRESS, WHO_AM_I_REG)
        print(f"Device ID: {device_id:#02x}")
        


        bus.write_byte_data(MAGNETOMETER_ADDRESS, CTRL1, 0x01)  
        print("Magnetometer initialized.")
    except Exception as e:
        print(f"Error initializing magnetometer: {e}")

def read_magnetometer_data():
    """Read and return X, Y, Z magnetometer data."""
    try:
        
        x_lsb = bus.read_byte_data(MAGNETOMETER_ADDRESS, DATA_X_LSB)
        x_msb = bus.read_byte_data(MAGNETOMETER_ADDRESS, DATA_X_MSB)
        y_lsb = bus.read_byte_data(MAGNETOMETER_ADDRESS, DATA_Y_LSB)
        y_msb = bus.read_byte_data(MAGNETOMETER_ADDRESS, DATA_Y_MSB)
        z_lsb = bus.read_byte_data(MAGNETOMETER_ADDRESS, DATA_Z_LSB)
        z_msb = bus.read_byte_data(MAGNETOMETER_ADDRESS, DATA_Z_MSB)

       
        x = (x_msb << 8) | x_lsb
        y = (y_msb << 8) | y_lsb
        z = (z_msb << 8) | z_lsb

       
        if x >= 0x8000:
            x -= 0x10000
        if y >= 0x8000:
            y -= 0x10000
        if z >= 0x8000:
            z -= 0x10000

        return x, y, z
    except Exception as e:
        print(f"Error reading magnetometer data: {e}")
        return None, None, None

if __name__ == "__main__":
 
    initialize_ist8308()

    
    try:
        while True:
            x, y, z = read_magnetometer_data()
            if x is not None:
                print(f"Magnetic field (X, Y, Z): {x}, {y}, {z}")
            time.sleep(0.5)  
    except KeyboardInterrupt:
        print("Terminating program.")
    finally:
        bus.close()
