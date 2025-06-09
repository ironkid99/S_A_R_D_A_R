import Adafruit_DHT
import RPi.GPIO as GPIO
import smbus
import math
import time
# import cv2
# import face_recognition
import threading
import paho.mqtt.client as mqtt
import json

# MQTT Broker Settings
broker_address = "test.mosquitto.org"
broker_port = 1883
topic = "Data_SARDAR"

# Create MQTT client instance
client = mqtt.Client()

# Connect to MQTT broker
client.connect(broker_address, broker_port)

bus = smbus.SMBus(1)  

MPU9250_ADDRESS = 0x68  
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
MAG_XOUT_L = 0x03


DHT_PIN = 17
TRIG_PIN = 27
ECHO_PIN = 22

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DHT_PIN, GPIO.IN)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

def get_temperature_humidity():
    humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, DHT_PIN)
    if humidity is not None and temperature is not None:
#         print('Temperature={0:0.1f}C  Humidity={1:0.1f}%'.format(temperature, humidity))
        return temperature, humidity
    else:
        print('Failed to retrieve data from DHT sensor')
        return None, None

def get_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    end_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    duration = end_time - start_time
    distance = duration * 34300 / 2  
    return round(distance, 2)

def read_acceleration():
    data = bus.read_i2c_block_data(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]


    if accel_x > 32767:
        accel_x -= 65536
    if accel_y > 32767:
        accel_y -= 65536
    if accel_z > 32767:
        accel_z -= 65536


    accel_x = accel_x / 16384.0
    accel_y = accel_y / 16384.0
    accel_z = accel_z / 16384.0

    return accel_x, accel_y, accel_z

def read_gyroscope():
    data = bus.read_i2c_block_data(MPU9250_ADDRESS, GYRO_XOUT_H, 6)
    gyro_x = (data[0] << 8) | data[1]
    gyro_y = (data[2] << 8) | data[3]
    gyro_z = (data[4] << 8) | data[5]

   
    if gyro_x > 32767:
        gyro_x -= 65536
    if gyro_y > 32767:
        gyro_y -= 65536
    if gyro_z > 32767:
        gyro_z -= 65536

    
    gyro_x = gyro_x / 131.0
    gyro_y = gyro_y / 131.0
    gyro_z = gyro_z / 131.0

    return gyro_x, gyro_y, gyro_z

def read_magnetometer():
    data = bus.read_i2c_block_data(MPU9250_ADDRESS, MAG_XOUT_L, 6)
    mag_x = (data[1] << 8) | data[0]
    mag_y = (data[3] << 8) | data[2]
    mag_z = (data[5] << 8) | data[4]

 
    if mag_x > 32767:
        mag_x -= 65536
    if mag_y > 32767:
        mag_y -= 65536
    if mag_z > 32767:
        mag_z -= 65536

    
    mag_x = mag_x * 0.15
    mag_y = mag_y * 0.15
    mag_z = mag_z * 0.15

    return mag_x, mag_y, mag_z

def complementary_filter(prev_pitch, prev_roll, prev_yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, dt):
    alpha = 0.98 


    pitch_acc = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
    roll_acc = math.atan2(-accel_x, accel_z)


    pitch_gyro = prev_pitch + gyro_y * dt
    roll_gyro = prev_roll + gyro_x * dt

    
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc


    mag_x, mag_y, mag_z = read_magnetometer()
    yaw_mag = math.atan2(mag_y * math.cos(pitch) + mag_z * math.sin(pitch),
                         mag_x * math.cos(roll) + mag_y * math.sin(roll) * math.sin(pitch) + mag_z * math.sin(roll) * math.cos(pitch))


    yaw = alpha * (prev_yaw + gyro_z * dt) + (1 - alpha) * yaw_mag

    return pitch, roll, yaw


prev_pitch = 0
prev_roll = 0
prev_yaw = 0
# 
# def face_recognition_thread():
#     reference_images = ["rr.jpg"]
#     reference_face_encodings = []
# 
#     for image_path in reference_images:
#         reference_image = face_recognition.load_image_file(image_path)
#         
#         face_locations = face_recognition.face_locations(reference_image)
#         if len(face_locations) > 0:
#             
#             reference_face_encoding = face_recognition.face_encodings(reference_image)[0]
#             reference_face_encodings.append(reference_face_encoding)
#         else:
#             print(f"No face detected in {image_path}")
# 
#     cap = cv2.VideoCapture(0)  
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break
# 
#         frame = cv2.resize(frame, (600, 400))
# 
#         face_locations = face_recognition.face_locations(frame)
#         face_encodings = face_recognition.face_encodings(frame, face_locations)
# 
#         for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
# 
#             match_status = [False] * len(reference_images)
# 
#             
#             for i, reference_face_encoding in enumerate(reference_face_encodings):
#                 match_status[i] = face_recognition.compare_faces([reference_face_encoding], face_encoding)[0]
# 
#             cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
# 
#             if any(match_status):
#                 cv2.putText(frame, "Intruder Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
# 
#         cv2.imshow('Real-time Face Recognition', frame)
# 
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
# 
#     cap.release()
#     cv2.destroyAllWindows()

def publish_sensor_data(data):
    client.publish(topic, data)
#     print("Sensor data sent")

if __name__ == '__main__':
    try:
        setup()
#         face_thread = threading.Thread(target=face_recognition_thread)
#         face_thread.start()

        while True:
            temperature, humidity = get_temperature_humidity()
            distance = get_distance()
#             print("Distance:", distance, "cm")
            accel_x, accel_y, accel_z = read_acceleration()
            gyro_x, gyro_y, gyro_z = read_gyroscope()
            dt = 0.1  
            pitch, roll, yaw = complementary_filter(prev_pitch, prev_roll, prev_yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, dt)
            prev_pitch = pitch
            prev_roll = roll
            prev_yaw = yaw
#             print(roll,pitch,yaw)

            # Format sensor data
            sensor_data = {
                "temperature": temperature,
                "humidity": humidity,
                "distance": distance,
                "acceleration": {
                    "x": accel_x,
                    "y": accel_y,
                    "z": accel_z
                },
                "gyroscope": {
                    "x": gyro_x,
                    "y": gyro_y,
                    "z": gyro_z
                },
                "orientation": {
                   
                    "roll": math.degrees(roll),
                    "pitch": math.degrees(pitch),
                    "yaw": math.degrees(yaw)
                }
            }

            # Publish sensor data as JSON
            publish_sensor_data(json.dumps(sensor_data))
            
            print("Sensor data sent:", sensor_data)

            # Sleep for 5 minutes
            time.sleep(30)

            # Sleep for a while
            time.sleep(dt)

    except KeyboardInterrupt:
        GPIO.cleanup()
