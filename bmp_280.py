import time
from bmp280 import BMP280
from smbus2 import SMBus
bus = SMBus(4)
bmp280 = BMP280(i2c_dev=bus)

while True:
          temperature = bmp280.get_temperature()
          pressure = bmp280.get_pressure()
          degree_sign = u"\N{DEGREE SIGN}"
          format_temp = "{:.2f}".format(temperature)
          print('Temperature = ' + format_temp + degree_sign + 'C')
          format_press = "{:.2f}".format(pressure)
          print('Pressure = ' + format_press + ' hPa \n')
          time.sleep(4)