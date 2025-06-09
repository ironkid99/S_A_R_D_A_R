# # import RPi.GPIO as GPIO
# # import time
# # 
# # # Pin definitions
# # IN1 = 26
# # IN2 = 25
# # IN3 = 29
# # IN4 = 28
# # ENA = 31
# # ENB = 11
# # 
# # # Set up GPIO
# # GPIO.setmode(GPIO.BCM)
# # GPIO.setup(IN1, GPIO.OUT)
# # GPIO.setup(IN2, GPIO.OUT)
# # GPIO.setup(IN3, GPIO.OUT)
# # GPIO.setup(IN4, GPIO.OUT)
# # GPIO.setup(ENA, GPIO.OUT)
# # GPIO.setup(ENB, GPIO.OUT)
# # 
# # # Set up PWM on the enable pins
# # pwmA = GPIO.PWM(ENA, 1000) # Frequency: 1kHz
# # pwmB = GPIO.PWM(ENB, 1000) # Frequency: 1kHz
# # 
# # # Start PWM with a duty cycle of 0 (stop)
# # pwmA.start(0)
# # pwmB.start(0)
# # 
# # def set_speed(pwm, speed):
# #     """Set the speed of the motor."""
# #     pwm.ChangeDutyCycle(speed)
# # 
# # def move_forward():
# #     """Move both motors forward."""
# #     GPIO.output(IN1, GPIO.HIGH)
# #     GPIO.output(IN2, GPIO.LOW)
# #     GPIO.output(IN3, GPIO.HIGH)
# #     GPIO.output(IN4, GPIO.LOW)
# # 
# # def move_backward():
# #     """Move both motors backward."""
# #     GPIO.output(IN1, GPIO.LOW)
# #     GPIO.output(IN2, GPIO.HIGH)
# #     GPIO.output(IN3, GPIO.LOW)
# #     GPIO.output(IN4, GPIO.HIGH)
# # 
# # def stop():
# #     """Stop both motors."""
# #     GPIO.output(IN1, GPIO.LOW)
# #     GPIO.output(IN2, GPIO.LOW)
# #     GPIO.output(IN3, GPIO.LOW)
# #     GPIO.output(IN4, GPIO.LOW)
# # 
# # try:
# #     while True:
# #         # Move forward
# #         set_speed(pwmA, 75)  # 75% speed
# #         set_speed(pwmB, 75)  # 75% speed
# #         move_forward()
# #         time.sleep(2)
# # 
# #         # Stop
# #         stop()
# #         time.sleep(1)
# # 
# #         # Move backward
# #         set_speed(pwmA, 75)
# #         set_speed(pwmB, 75)
# #         move_backward()
# #         time.sleep(2)
# # 
# #         # Stop
# #         stop()
# #         time.sleep(1)
# # 
# # except KeyboardInterrupt:
# #     pass
# # 
# # # Cleanup
# # pwmA.stop()
# # pwmB.stop()
# # GPIO.cleanup()
# #
# 
# 
# import RPi.GPIO as GPIO
# import time
# 
# # Pin definitions for motor control
# IN1 = 26
# IN2 = 19
# IN3 = 13
# IN4 = 12
# ENA = 21
# ENB = 20
# 
# # Pin definitions for ultrasonic sensor
# TRIG_PIN = 27
# ECHO_PIN = 22
# 
# # Set up GPIO for motor control
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(IN1, GPIO.OUT)
# GPIO.setup(IN2, GPIO.OUT)
# GPIO.setup(IN3, GPIO.OUT)
# GPIO.setup(IN4, GPIO.OUT)
# GPIO.setup(ENA, GPIO.OUT)
# GPIO.setup(ENB, GPIO.OUT)
# 
# # Set up GPIO for ultrasonic sensor
# GPIO.setup(TRIG_PIN, GPIO.OUT)
# GPIO.setup(ECHO_PIN, GPIO.IN)
# 
# # Set up PWM on the enable pins
# pwmA = GPIO.PWM(ENA, 1000) # Frequency: 1kHz
# pwmB = GPIO.PWM(ENB, 1000) # Frequency: 1kHz
# 
# # Start PWM with a duty cycle of 0 (stop)
# pwmA.start(0)
# pwmB.start(0)
# 
# def set_speed(pwm, speed):
#     """Set the speed of the motor."""
#     pwm.ChangeDutyCycle(speed)
# 
# def move_forward():
#     """Move both motors forward."""
#     GPIO.output(IN1, GPIO.HIGH)
#     GPIO.output(IN2, GPIO.LOW)
#     GPIO.output(IN3, GPIO.HIGH)
#     GPIO.output(IN4, GPIO.LOW)
# 
# def move_backward():
#     """Move both motors backward."""
#     GPIO.output(IN1, GPIO.LOW)
#     GPIO.output(IN2, GPIO.HIGH)
#     GPIO.output(IN3, GPIO.LOW)
#     GPIO.output(IN4, GPIO.HIGH)
# 
# def stop():
#     """Stop both motors."""
#     GPIO.output(IN1, GPIO.LOW)
#     GPIO.output(IN2, GPIO.LOW)
#     GPIO.output(IN3, GPIO.LOW)
#     GPIO.output(IN4, GPIO.LOW)
# 
# def get_distance():
#     """Measure distance using the ultrasonic sensor."""
#     GPIO.output(TRIG_PIN, True)
#     time.sleep(0.00001)  # 10 microseconds
#     GPIO.output(TRIG_PIN, False)
# 
#     start_time = time.time()
#     end_time = time.time()
# 
#     while GPIO.input(ECHO_PIN) == 0:
#         start_time = time.time()
# 
#     while GPIO.input(ECHO_PIN) == 1:
#         end_time = time.time()
# 
#     duration = end_time - start_time
#     distance = (duration * 34300) / 2  # Speed of sound = 343 m/s
#     return round(distance, 2)
# 
# try:
#     while True:
#         distance = get_distance()
#         print("Measured Distance = {:.1f} cm".format(distance))
#         
#         if distance < 10:
#             stop()
#             
#             move_backward()
#             print("Object detected within 10 cm! Stopping motors.")
#         else:
#             set_speed(pwmA, 75)  # 75% speed
#             set_speed(pwmB, 75)  # 75% speed
#             move_forward()
#             print("Moving forward.")
#         
#         time.sleep(0.1)  # Check distance every 0.1 seconds
# 
# except KeyboardInterrupt:
#     print("Measurement stopped by User")
# finally:
# # Cleanup
#     pwmA.stop()
#     pwmB.stop()
#     GPIO.cleanup()
#




import RPi.GPIO as GPIO
import time

# Pin definitions for motor control
IN1 = 26
IN2 = 19
IN3 = 13
IN4 = 12
ENA = 21
ENB = 20

# Pin definitions for ultrasonic sensor
TRIG_PIN = 27
ECHO_PIN = 22

# Set up GPIO for motor control
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up GPIO for ultrasonic sensor
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Set up PWM on the enable pins
pwmA = GPIO.PWM(ENA, 1000) # Frequency: 1kHz
pwmB = GPIO.PWM(ENB, 1000) # Frequency: 1kHz

# Start PWM with a duty cycle of 0 (stop)
pwmA.start(0)
pwmB.start(0)

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

def stop():
    """Stop both motors."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def get_distance():
    """Measure distance using the ultrasonic sensor."""
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    end_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    duration = end_time - start_time
    distance = (duration * 34300) / 2  # Speed of sound = 343 m/s
    return round(distance, 2)

def move_backward_for_distance(distance_cm, speed=75):
    """Move the robot backward for a specific distance."""
    set_speed(pwmA, speed)
    set_speed(pwmB, speed)
    move_backward()
    # Calculate the time to move backward based on speed
    # Assuming speed is in percentage of max speed, we approximate time
    time_to_move = distance_cm / (speed / 100 * 34.3)  # 34.3 cm/s is an approximation
    time.sleep(time_to_move)
    stop()

try:
    while True:
        distance = get_distance()
        print("Measured Distance = {:.1f} cm".format(distance))
        
        if distance < 10:
            print("Object detected within 10 cm! Moving backward.")
            move_backward_for_distance(10)  # Move backward 10 cm
            print("Waiting for distance to be more than 50 cm.")
            while get_distance() <= 50:
                stop()
                time.sleep(0.1)
        else:
            set_speed(pwmA, 100)  # 75% speed
            set_speed(pwmB, 100)  # 75% speed
            move_forward()
            print("Moving forward.")
        
        time.sleep(0.1)  # Check distance every 0.1 seconds

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
finally:
    # Cleanup
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    