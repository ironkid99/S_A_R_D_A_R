import requests
import Adafruit_DHT
import json
import time

sensor = Adafruit_DHT.DHT11
pin = 4
SERVER_URL = "http://192.168.0.102/"

while True:
    humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

    if humidity is not None and temperature is not None:
        print("Temperature={0:0.1f}C  Humidity={1:0.1f}%".format(temperature, humidity))

        # Prepare data to be sent to the server
        data = {"temperature": temperature, "humidity": humidity}

        try:
            # Send POST request to the Apache server
            response = requests.post(SERVER_URL, json=data)

            # Check if the request was successful
            if response.status_code == 200:
                print("Data posted successfully to server")
            else:
                print(
                    "Failed to post data to server. Status code:", response.status_code
                )
        except Exception as e:
            print("Error posting data to server:", str(e))
    else:
        print("Failed to retrieve data from DHT sensor")

    time.sleep(0.1)
