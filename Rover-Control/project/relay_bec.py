import Jetson.GPIO as GPIO
import time

# Turn off warning
GPIO.setwarnings(False)

# Use physical pin numbers
GPIO.setmode(GPIO.BOARD)

# Define the pin
RELAY_PIN = 40  # Pin 40 is GPIO21 (BCM)

# Set up the pin as output
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)

try:
    print("Turning relay ON")
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    GPIO.output(37, GPIO.HIGH)	
    time.sleep(3)
    print("Turning relay OFF")
    GPIO.output(RELAY_PIN, GPIO.LOW)
    GPIO.output(37, GPIO.LOW)
finally:
    print("Cleaning up GPIO")
    GPIO.cleanup()



