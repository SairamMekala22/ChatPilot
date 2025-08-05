import Jetson.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Use GPIO 17 (physical pin 11)
relay_pin = 26

# Set pin as output
GPIO.setup(relay_pin, GPIO.OUT)

print("Relay control started. Press Ctrl+C to stop.")

try:
    while True:
        # Turn relay ON
        GPIO.output(relay_pin, GPIO.HIGH)  # LOW for active-low relay
        print("Relay ON")
        time.sleep(3)

        # Turn relay OFF
        GPIO.output(relay_pin, GPIO.LOW)
        print("Relay OFF")
        time.sleep(3)

except KeyboardInterrupt:
    print("Stopped by User")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up")

