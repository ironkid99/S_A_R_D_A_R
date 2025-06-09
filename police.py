import RPi.GPIO as GPIO
import time

# Use BCM pin numbering
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the LEDs
led_pins = [23, 24, 25]

# Set up GPIO pins as outputs
for pin in led_pins:
    GPIO.setup(pin, GPIO.OUT)

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

if __name__ == "__main__":
    police_light()
