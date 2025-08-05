from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/ttyACM1', baud=57600, wait_ready=False)
print(vehicle.heading)
count = 1
@vehicle.on_message('SERVO_OUTPUT_RAW')
def servo_listener(self, name, message):
    global latest_servo1_value, latest_servo3_value, count 
    latest_servo1_value = message.servo1_raw
    latest_servo3_value = message.servo3_raw
    count += 1 
    print(f"[SERVO] Servo1: {latest_servo1_value}, Servo3: {latest_servo3_value}, count : {count}")

#time.sleep(10)
print("Overriding....")
#vehicle.channels.overrides = {'1': 1500, '2': 1600}
print("Reset")
#vehicle.channels.overrides = {}
#time.sleep(10)


vehicle.close()
