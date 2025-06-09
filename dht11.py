# import Adafruit_DHT
# 
# sensor = Adafruit_DHT.DHT11
# pin = 4  # GPIO pin number
# 
# humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
# 
# if humidity is not None and temperature is not None:
#     print('Temperature={0:0.1f}C  Humidity={1:0.1f}%'.format(temperature, humidity))
# else:
#     print('Failed to retrieve data from DHT sensor')
#

import Adafruit_DHT
import time

sensor = Adafruit_DHT.DHT11
pin =  17

while True:
    humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

    if humidity is not None and temperature is not None:
        print('Temperature={0:0.1f}C  Humidity={1:0.1f}%'.format(temperature, humidity))
    else:
        print('Failed to retrieve data from DHT sensor')

   
    time.sleep(0.1)  
