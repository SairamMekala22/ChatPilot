import Jetson.GPIO as GPIO
import time
from dronekit import connect, VehicleMode

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Use GPIO 17 (physical pin 11)
relay_pin = 26
latest_servo4_value = 0
# Set pin as output
GPIO.setup(relay_pin, GPIO.OUT)

print("[Info] Connecting to Pixhawk...")
# vehicle = connect('COM3', wait_ready=True, baud=57600)
vehicle = connect('/dev/ttyACM1', wait_ready=True, baud=57600)
print("[Info] Connected to Pixhawk.")

@vehicle.on_message('SERVO_OUTPUT_RAW')
def servo_listener(self, name, message):
    global latest_servo4_value 
    # latest_servo1_value = message.servo1_raw  # Right-side wheels
    latest_servo4_value = message.servo4_raw  # Left-side wheels
    print(f"[Servo Output] servo 4: {latest_servo4_value}")

print("Relay control started. Press Ctrl+C to stop.")

try:
    # while True:
    #     # Turn relay ON
    #     GPIO.output(relay_pin, GPIO.HIGH)  # LOW for active-low relay
    #     print("Relay ON")
    #     time.sleep(3)

    #     # Turn relay OFF
    #     GPIO.output(relay_pin, GPIO.LOW)
    #     print("Relay OFF")
    #     time.sleep(3)
    while True:
        if latest_servo4_value > 1500:
            GPIO.output(relay_pin, GPIO.HIGH) 
            print("on")    
        else:
            GPIO.output(relay_pin, GPIO.LOW) 
            print("off")


        time.sleep(2)
except KeyboardInterrupt:
    print("Stopped by User")

finally:
    # GPIO.cleanup()
    vehicle.close()
    print('vehicle disconnected')
    print("GPIO cleaned up")

