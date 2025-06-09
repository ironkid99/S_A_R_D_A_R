# import RPi.GPIO as GPIO
# import time
# 
# # Set GPIO Pins
# GPIO_TRIGGER = 27
# GPIO_ECHO = 22
# 
# # Set GPIO direction (IN / OUT)
#GPIO.setmode(GPIO.BCM)
# GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
# GPIO.setup(GPIO_ECHO, GPIO.IN)
# 
# def distance():
#     # Set trigger to HIGH
#     GPIO.output(GPIO_TRIGGER, True)
# 
#     # Set trigger after 0.01ms to LOW
#     time.sleep(0.00001)
#     GPIO.output(GPIO_TRIGGER, False)
# 
#     StartTime = time.time()
#     StopTime = time.time()
# 
#     # Save StartTime
#     while GPIO.input(GPIO_ECHO) == 0:
#         StartTime = time.time()
# 
#     # Save time of arrival
#     while GPIO.input(GPIO_ECHO) == 1:
#         StopTime = time.time()
# 
#     # Time difference between start and arrival
#     TimeElapsed = StopTime - StartTime
#     # Multiply with the speed of sound (34300 cm/s)
#     # and divide by 2, because it's the time to the object and back
#     distance = (TimeElapsed * 34300) / 2
# 
#     return distance
# 
# if __name__ == '__main__':
#     try:
#         while True:
#             dist = distance()
#             print ("Measured Distance = %.1f cm" % dist)
#             time.sleep(1)
# 
#         # Reset by pressing CTRL + C
#     except KeyboardInterrupt:
#         print("Measurement stopped by User")
#         GPIO.cleanup()
#

import RPi.GPIO as GPIO
import time

# Set GPIO pin numbers
TRIG_PIN = 27
ECHO_PIN = 22

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.000001)
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    end_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    duration = end_time - start_time
    distance = duration * 34300 / 2  # Speed of sound = 343 m/s
    return round(distance , 2)
#     return distance

def cleanup():
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        setup()
        while True:
            distance = get_distance()
            print("Distance:", distance, "cm")
            time.sleep(1)
    except KeyboardInterrupt:
        cleanup()

