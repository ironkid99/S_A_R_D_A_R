import smbus
import math
import time

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

def read_acceleration():
    data = bus.read_i2c_block_data(MPU9250_ADDRESS, ACCEL_XOUT_H, 6)
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]

    # Convert to signed values
    if accel_x > 32767:
        accel_x -= 65536
    if accel_y > 32767:
        accel_y -= 65536
    if accel_z > 32767:
        accel_z -= 65536

    # Convert to m/s^2
    accel_x = accel_x / ACCEL_SCALE
    accel_y = accel_y / ACCEL_SCALE
    accel_z = accel_z / ACCEL_SCALE

    return accel_x, accel_y, accel_z

def read_gyroscope():
    data = bus.read_i2c_block_data(MPU9250_ADDRESS, GYRO_XOUT_H, 6)
    gyro_x = (data[0] << 8) | data[1]
    gyro_y = (data[2] << 8) | data[3]
    gyro_z = (data[4] << 8) | data[5]

    # Convert to signed values
    if gyro_x > 32767:
        gyro_x -= 65536
    if gyro_y > 32767:
        gyro_y -= 65536
    if gyro_z > 32767:
        gyro_z -= 65536

    # Convert to degrees/s
    gyro_x = gyro_x / GYRO_SCALE
    gyro_y = gyro_y / GYRO_SCALE
    gyro_z = gyro_z / GYRO_SCALE

    return gyro_x, gyro_y, gyro_z

def read_magnetometer():
    data = bus.read_i2c_block_data(MPU9250_ADDRESS, MAG_XOUT_L, 6)
    mag_x = (data[1] << 8) | data[0]
    mag_y = (data[3] << 8) | data[2]
    mag_z = (data[5] << 8) | data[4]

    # Convert to signed values
    if mag_x > 32767:
        mag_x -= 65536
    if mag_y > 32767:
        mag_y -= 65536
    if mag_z > 32767:
        mag_z -= 65536

    # Convert to uT
    mag_x = mag_x * MAG_SCALE
    mag_y = mag_y * MAG_SCALE
    mag_z = mag_z * MAG_SCALE

    return mag_x, mag_y, mag_z

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

    # Yaw calculation using magnetometer data
    mag_x, mag_y, mag_z = read_magnetometer()
    yaw_mag = math.atan2(mag_y * math.cos(pitch) + mag_z * math.sin(pitch),
                         mag_x * math.cos(roll) + mag_y * math.sin(roll) * math.sin(pitch) + mag_z * math.sin(roll) * math.cos(pitch))

    # Apply complementary filter to yaw angle
    yaw = alpha * (prev_yaw + gyro_z * dt) + (1 - alpha) * yaw_mag

    return pitch, roll, yaw

# Initial values
prev_pitch = 0
prev_roll = 0
prev_yaw = 0

while True:
    accel_x, accel_y, accel_z = read_acceleration()
    gyro_x, gyro_y, gyro_z = read_gyroscope()

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
