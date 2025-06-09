import Adafruit_DHT
import RPi.GPIO as GPIO
import smbus
import math
import time
import cv2
import face_recognition
import threading
import paho.mqtt.client as mqtt
import jsonFinal
import serial
from threading import Thread
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit
# import RPi.GPIO as GPIO

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

IN1 = 26
IN2 = 19
IN3 = 13
IN4 = 12
ENA = 21
ENB = 20


DHT_PIN = 17
TRIG_PIN = 27
ECHO_PIN = 22

led_pins = [23,24,25]

# Configure the serial port for GPS
ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

# Suppress GPIO warnings
GPIO.setwarnings(False)

def setup():

    GPIO.setmode(GPIO.BCM)
   
    for pin in led_pins:
        GPIO.setup(pin, GPIO.OUT)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(ENA, GPIO.OUT)
    GPIO.setup(ENB, GPIO.OUT)
    GPIO.setup(DHT_PIN, GPIO.IN)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
   
    pwmA = GPIO.PWM(ENA, 1000)  # Frequency: 1kHz
    pwmB = GPIO.PWM(ENB, 1000)

    pwmA.start(0)
    pwmB.start(0)

self_drive_thread = None
self_drive_stop = False

def set_speed(pwm, speed):
    """Set the speed of the motor."""
    pwm.ChangeDutyCycle(speed)

def move_forward():
    """Move both motors forward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def move_backward():
    """Move both motors backward."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def move_left():
    """Move left motor backward and right motor forward."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def move_right():
    """Move right motor backward and left motor forward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    """Stop both motors."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def get_temperature_humidity():
    humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, DHT_PIN)
    if humidity is not None and temperature is not None:
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

def move_backward_for_distance(distance_cm, speed=75):
    """Move the robot backward for a specific distance."""
    set_speed(pwmA, speed)
    set_speed(pwmB, speed)
    move_backward()
    time_to_move = distance_cm / (speed / 100 * 34.3)  # 34.3 cm/s is an approximation
    time.sleep(time_to_move)
    stop()



def self_drive_logic():
    global self_drive_stop
    while not self_drive_stop:
        distance = get_distance()
        if distance > 10:
            set_speed(pwmA, 100)  # 100% speed
            set_speed(pwmB, 100)  # 100% speed
            move_forward()
        else:
            move_backward_for_distance(10)
        time.sleep(0.1)
    stop()

def start_self_drive():
    global self_drive_thread, self_drive_stop
    self_drive_stop = False
    self_drive_thread = Thread(target=self_drive_logic)
    self_drive_thread.start()

def stop_self_drive():
    global self_drive_stop
    self_drive_stop = True
    if self_drive_thread:
        self_drive_thread.join()

# Create Flask app
app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def index():
    return render_template_string('''
<!DOCTYPE html>
<html>

<head>
    <title>Motor Control</title>
    <script src="https://cdn.socket.io/4.0.0/socket.io.min.js"></script>
    <style>
        * {
            background-color: #28303c;
        }
        #heading {
            background-color: #28303c;
            height: 150px;
            width: 100%;
        }
        .header {
            display: flex;
            align-items: center;
            justify-content: center;
            color: #53dfd5;
            font-size: 40px;
            font-family: 'Noto Serif', serif;
        }
        .title {
            margin-top: 10px;
            margin-bottom: 5px;
        }
        .para {
            margin-top: 5px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-size: 30px;
            font-family: 'PT Serif', serif;
            margin-left: 150px;
            margin-right: 150px;
            text-align: center;
            margin-bottom: 5px;
        }
        body {
            font-family: 'Noto Serif', serif;
            font-size: 30px;
            text-align: center;
            margin: 0;
            padding: 0;
            color: #53dfd5;
        }
       
        .topic {
            margin-top: 15px;
            margin-bottom: 20px;
        }
        .remote {
            display: inline-block;
            height: 370px;
            width: 330px ;
            border: 3px solid #ccc;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
        }
        .button {
            display: inline-block;
            padding: 15px 30px;
            margin: 10px;
            font-size: 18px;
            background-color: #4CAF50;
            border: 2px solid #4CAF50;
            color: #fff;
            border-radius: 10px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .button:hover {
            background-color: #45a049;
            opacity: 0.8;
        }
        .top{
            margin-top: 2px;
            margin-bottom: 2px;
        }
        .middle{
            margin-top: 2px;
            margin-bottom: 2px;
        }
        .bottom{
            margin-top: 2px;
            margin-bottom: 2px;
        }
        .self{
            margin-top: 5px;
            margin-bottom: 2px;
        }
        .drive{
            font-size: 30px;
            font-family: 'Noto Serif', serif ;
        }
        input[type=range] {
            width: 80%;
            margin: 20px auto;
            display: block;
        }
        #status {
            margin-top: 20px;
            font-weight: bold;
        }

    </style>
    <script>
        var socket = io();

        function sendCommand(direction, speed) {
            socket.emit('move', { direction: direction, speed: speed });
        }

        function forward() {
            let speed = document.getElementById('speed').value;
            sendCommand('forward', speed);
        }

        function backward() {
            let speed = document.getElementById('speed').value;
            sendCommand('backward', speed);
        }

        function left() {
            let speed = document.getElementById('speed').value;
            sendCommand('left', speed);
        }

        function right() {
            let speed = document.getElementById('speed').value;
            sendCommand('right', speed);
        }

        function stop() {
            sendCommand('stop', 0);
        }

        function startSelfDriving() {
            socket.emit('self_drive');
        }

        socket.on('response', function (data) {
            document.getElementById('status').innerText = 'Command sent successfully!';
            setTimeout(function () {
                document.getElementById('status').innerText = '';
            }, 2000);
        });
    </script>
</head>

<body>
    <div id="heading">
        <div class="header">
            <h1 class="title">SARDAR</h1>
        </div>
        <div class="name">
            <p class="para"> Surveillance with Artificial Intelligence & Remote Control based on Deep Learning Advanced
                Robot </p>
        </div>
    </div>
    <h1 class="topic">Remote Control</h1>
    <div class="remote">
        <input type="range" id="speed" min="0" max="100" value="75">
        <div class="top">
            <button class="button" onclick="forward()">▲</button><br>
        </div>
        <div class="middle">
            <button class="button" onclick="left()">◀</button>
            <button class="button" onclick="stop()">■</button>
            <button class="button" onclick="right()">▶</button>
        </div>
        <div class="bottom">
            <button class="button" onclick="backward()">▼</button><br>
        </div>
        <div class="self">
        <button class="button drive" onclick="startSelfDriving()">AUTO</button>
    </div>
    </div>
    <div id="status"></div>
</body>

</html>
    ''')

@socketio.on('move')
def handle_move(data):
    global self_drive_stop
    if self_drive_thread and self_drive_thread.is_alive():
        stop_self_drive()
    direction = data['direction']
    speed = int(data['speed'])
    set_speed(pwmA, speed)
    set_speed(pwmB, speed)
    if direction == 'forward':
        move_forward()
    elif direction == 'backward':
        move_backward()
    elif direction == 'left':
        move_left()
    elif direction == 'right':
        move_right()
    elif direction == 'stop':
        stop()
    emit('response', {'status': 'ok'})

@socketio.on('self_drive')
def handle_self_drive():
    global self_drive_thread
    if not self_drive_thread or not self_drive_thread.is_alive():
        start_self_drive()
    emit('response', {'status': 'Self-driving started'})



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

def police_light():
    try:
        while True:
            # Turn on LEDs 23 and 24 (red)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.HIGH)
            time.sleep(0.1)
           
            # Turn off LEDs 23 and 24 (red)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
           
            # Turn on LED 25 (blue)
            GPIO.output(25, GPIO.HIGH)
            time.sleep(0.1)
           
            # Turn off LED 25 (blue)
            GPIO.output(25, GPIO.LOW)

            # Repeat the above steps to create a blinking effect
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Cleanup GPIO settings before exit
        GPIO.cleanup()

def run_police_light():
    thread = Thread(target=police_light)
    thread.daemon = True  # Daemonize thread to exit when main program exits
    thread.start()

def face_recognition_thread():
    reference_images = ["rr.jpg"]
    reference_face_encodings = []

    for image_path in reference_images:
        reference_image = face_recognition.load_image_file(image_path)

        face_locations = face_recognition.face_locations(reference_image)
        if len(face_locations) > 0:
            reference_face_encoding = face_recognition.face_encodings(reference_image)[0]
            reference_face_encodings.append(reference_face_encoding)
        else:
            print(f"No face detected in {image_path}")

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (600, 400))

        face_locations = face_recognition.face_locations(frame)
        face_encodings = face_recognition.face_encodings(frame, face_locations)

        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):

            match_status = [False] * len(reference_images)

            for i, reference_face_encoding in enumerate(reference_face_encodings):
                match_status[i] = face_recognition.compare_faces([reference_face_encoding], face_encoding)[0]

            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            if any(match_status):
                cv2.putText(frame, "Intruder Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('Real-time Face Recognition', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def publish_sensor_data(data):
    client.publish(topic, data)

def read_gps_data():
    try:
        while True:
            nmea_sentence = ser.readline().decode('utf-8').strip()
            if nmea_sentence.startswith('$GPGGA'):
                fields = nmea_sentence.split(',')
                if len(fields) >= 10:
                    latitude = fields[2]
                    longitude = fields[4]
                    lat_hemisphere = fields[3]
                    lon_hemisphere = fields[5]

                    latitude_dec = float(latitude[:2]) + float(latitude[2:]) / 60
                    longitude_dec = float(longitude[:3]) + float(longitude[3:]) / 60

                    if lat_hemisphere == 'S':
                        latitude_dec *= -1
                    if lon_hemisphere == 'W':
                        longitude_dec *= -1

                    return latitude_dec, longitude_dec
            return 0, 0
    except KeyboardInterrupt:
        ser.close()
        print("GPS script stopped")
        return 0, 0

if __name__ == '__main__':
#     socketio.run(app, host='0.0.0.0', port=5000)
    try:
        setup()
        run_police_light()
        face_thread = threading.Thread(target=face_recognition_thread)
        face_thread.start()

        while True:
            temperature, humidity = get_temperature_humidity()
            distance = get_distance()
            accel_x, accel_y, accel_z = read_acceleration()
            gyro_x, gyro_y, gyro_z = read_gyroscope()
            dt = 0.1
            pitch, roll, yaw = complementary_filter(prev_pitch, prev_roll, prev_yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, dt)
            prev_pitch = pitch
            prev_roll = roll
            prev_yaw = yaw

            latitude, longitude = read_gps_data()

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
                },
                "gps": {
                    "latitude": latitude,
                    "longitude": longitude
                }
            }

            publish_sensor_data(json.dumps(sensor_data))

            print("Sensor data sent:", sensor_data)

            time.sleep(30)

    except KeyboardInterrupt:
        GPIO.cleanup()
        ser.close()
       
    finally:
        stop_self_drive()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
