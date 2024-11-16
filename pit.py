import smbus
import time

MS4525DO_I2C_ADDR = 0x28
bus = smbus.SMBus(1)

def read_pressure_temperature():
    try:
        data = bus.read_i2c_block_data(MS4525DO_I2C_ADDR, 0, 4)

        pressure_raw = ((data[0] & 0x3F) << 8 | data[1]
        pressure = (pressure_raw - 8192) / 16384.0
        #temp_raw = ((data[2] << 8 | (data[3] & 0xE0)) >> 5
        #temperature = (temp_raw / 2047.0) * 200.0 - 50.0  # Temperature in Celsius

        return pressure
    except Exception as e:
        print(f"Error reading from MS4525DO: {e}")
        return None, None

if name == "__main__":
    while True:
        pressure = read_pressure_temperature()
        
        print(f"Pressure: {pressure:.4f}")
        time.sleep(1)