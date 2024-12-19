import smbus
import time

IST8308_I2C_ADDRESS = 0x1E
REG_WAI = 0x00
REG_CNTL3 = 0x0C
REG_STAT = 0x02
REG_DATAXL = 0x03
REG_DATAXH = 0x04
REG_DATAYL = 0x05
REG_DATAYH = 0x06
REG_DATAZL = 0x07
REG_DATAZH = 0x08
CNTL3_SRST = 0x01
DRDY_MASK = 0x01

class IST8308:
    def __init__(self, bus_num=1):
        self.bus = smbus.SMBus(bus_num)

    def write_register(self, reg, value):
        self.bus.write_byte_data(IST8308_I2C_ADDRESS, reg, value)

    def read_register(self, reg):
        return self.bus.read_byte_data(IST8308_I2C_ADDRESS, reg)

    def read_registers(self, start_reg, length):
        return self.bus.read_i2c_block_data(IST8308_I2C_ADDRESS, start_reg, length)

    def reset(self):
        self.write_register(REG_CNTL3, CNTL3_SRST)
        time.sleep(0.05)

    def configure(self):
        self.write_register(REG_CNTL3, 0x00)

    def read_data(self):
        status = self.read_register(REG_STAT)
        if status & DRDY_MASK:
            data = self.read_registers(REG_DATAXL, 6)
            x = (data[1] << 8) | data[0]
            y = (data[3] << 8) | data[2]
            z = (data[5] << 8) | data[4]
            x = x - 65536 if x > 32767 else x
            y = y - 65536 if y > 32767 else y
            z = z - 65536 if z > 32767 else z
            return x, y, z
        return None

def main():
    sensor = IST8308()
    sensor.reset()
    sensor.configure()
    while True:
        data = sensor.read_data()
        if data:
            print(f"Magnetometer data: X={data[0]} Y={data[1]} Z={data[2]}")
        time.sleep(0.1)

if __name__ == "__main__":
    main()
