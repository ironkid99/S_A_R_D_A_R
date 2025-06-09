import cv2
import face_recognition
import Adafruit_DHT
import smbus
import math
import time
import threading

# Function to read temperature and humidity from DHT11 sensor
def read_dht_sensor(sensor, pin):
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

        if humidity is not None and temperature is not None:
            print('Temperature={0:0.1f}C  Humidity={1:0.1f}%'.format(temperature, humidity))
        else:
            print('Failed to retrieve data from DHT sensor')

        time.sleep(0.1)

# Function to read sensor data from MPU9250 and perform complementary filter
def read_mpu9250_and_filter():
    bus = smbus.SMBus(1)  # or 0 for older Raspberry Pi models

    MPU9250_ADDRESS = 0x68  # default I2C address
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    MAG_XOUT_L = 0x03

    # Wake up the MPU9250 and set it to the accelerometer, gyroscope, and magnetometer mode
    bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0)
    time.sleep(0.1)

    # MPU9250 sensor constants
    ACCEL_SCALE = 16384.0  # LSB/g (for +/- 2g full scale range)
    GYRO_SCALE = 131.0      # LSB/(°/s) (for +/- 250°/s full scale range)
    MAG_SCALE = 0.15        # LSB/uT (for +/- 4800uT full scale range)

    # Initial values
    prev_pitch = 0
    prev_roll = 0
    prev_yaw = 0

    while True:
        data = bus.read_i2c_block_data(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)
        accel_x = (data[0] << 8) | data[1]
        accel_y = (data[2] << 8) | data[3]
        accel_z = (data[4] << 8) | data[5]

        data = bus.read_i2c_block_data(MPU9250_ADDRESS, GYRO_XOUT_H, 6)
        gyro_x = (data[0] << 8) | data[1]
        gyro_y = (data[2] << 8) | data[3]
        gyro_z = (data[4] << 8) | data[5]

        # Convert to signed values
        if accel_x > 32767:
            accel_x -= 65536
        if accel_y > 32767:
            accel_y -= 65536
        if accel_z > 32767:
            accel_z -= 65536

        if gyro_x > 32767:
            gyro_x -= 65536
        if gyro_y > 32767:
            gyro_y -= 65536
        if gyro_z > 32767:
            gyro_z -= 65536

        # Convert to m/s^2
        accel_x = accel_x / ACCEL_SCALE
        accel_y = accel_y / ACCEL_SCALE
        accel_z = accel_z / ACCEL_SCALE

        # Convert to degrees/s
        gyro_x = gyro_x / GYRO_SCALE
        gyro_y = gyro_y / GYRO_SCALE
        gyro_z = gyro_z / GYRO_SCALE

        # Sample rate (time interval between readings)
        dt = 0.1  # Adjust as needed

        # Calculate pitch, roll, and yaw angles using complementary filter
        pitch, roll, yaw = complementary_filter(prev_pitch, prev_roll, prev_yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, dt)

        # Update previous values for the next iteration
        prev_pitch = pitch
        prev_roll = roll
        prev_yaw = yaw

        # Print angles with two decimal places
        print("Pitch: {:.2f} Roll: {:.2f} Yaw: {:.2f}".format(math.degrees(pitch), math.degrees(roll), math.degrees(yaw)))
        time.sleep(dt)

# Function to perform face recognition
def perform_face_recognition():
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

# Complementary filter function
def complementary_filter(prev_pitch, prev_roll, prev_yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, dt):
    alpha = 0.98  # Complementary filter coefficient

    # Calculate pitch and roll angles from accelerometer data
    pitch_acc = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
    roll_acc = math.atan2(-accel_x, accel_z)

    # Integrate gyroscope data for better accuracy
    pitch_gyro = prev_pitch + gyro_y * dt
    roll_gyro = prev_roll + gyro_x * dt

    # Combine accelerometer and gyroscope readings using complementary filter
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc

    # Yaw calculation using magnetometer data (You may include it here if needed)

    return pitch, roll

# Create and start threads for DHT sensor reading, MPU9250 data reading and filtering, and face recognition
dht_thread = threading.Thread(target=read_dht_sensor, args=(Adafruit_DHT.DHT11, 17))
mpu9250_thread = threading.Thread(target=read_mpu9250_and_filter)
face_recognition_thread = threading.Thread(target=perform_face_recognition)

dht_thread.start()
mpu9250_thread.start()
face_recognition_thread.start()
