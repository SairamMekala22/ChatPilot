# from dronekit import connect, VehicleMode, LocationGlobalRelative

# vehicle = connect('COM3', baud=57600, wait_ready=False)

# # point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
# point1 = LocationGlobalRelative(17.397205, 78.489941, 0)
# vehicle.simple_goto(point1)

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import json
import serial

CONNECTION_STRING = '/dev/ttyACM0'
LIDAR_PORT = '/dev/ttyUSB0'
DDSM_PORT = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200
MIN_DISTANCE = 500
# Target GPS coordinates
TARGET_LAT = 17.397268
TARGET_LON = 78.490529
ALTITUDE = 0  # For ArduRover, altitude can be zero

latest_servo1_value = None
latest_servo3_value = None


# Connect to the vehicle (adjust port and baudrate)
vehicle = connect(CONNECTION_STRING, baud=57600, wait_ready=False)

def get_distance_meters(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    # Map servo PWM (1000-2000 µs) to speed (-100 to 100)
    return int((servo_value - 1500) / 500 * 100)

# Connect to DDSM Serial
ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)

@vehicle.on_message('SERVO_OUTPUT_RAW')
def servo_listener(self, name, message):
    global latest_servo1_value, latest_servo3_value
    latest_servo1_value = message.servo1_raw
    latest_servo3_value = message.servo3_raw
    print(f"[SERVO] Servo1: {latest_servo1_value}, Servo3: {latest_servo3_value}")

# Wait for GPS fix
while not vehicle.location.global_relative_frame.lat:
    print("Waiting for GPS fix...")
    time.sleep(1)

# Set mode to GUIDED
print("Setting mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode.name != "GUIDED":
     print("Waiting for mode change...")
     time.sleep(1)

# Arm the vehicle
print("Arming...")
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

# Send GOTO command
target_location = LocationGlobalRelative(TARGET_LAT, TARGET_LON, ALTITUDE)
print(f"Going to: {TARGET_LAT}, {TARGET_LON}")
vehicle.simple_goto(target_location)

# Monitor progress
while True:
    servo1 = latest_servo1_value
    servo3 = latest_servo3_value
    speed_left = scale_servo_to_speed(servo1)
    speed_right = scale_servo_to_speed(servo3)
    speed_right = max(-100, min(100, speed_right))
    speed_left = max(-100, min(100, speed_left))
    print(f"Speed left: {speed_left}, right: {speed_right}")

    command_right = {
        "T": 10010,
        "id": 2,
        "cmd": -speed_right,  # reverse polarity for right wheel
        "act": 3
    }
    command_left = {
        "T": 10010,
        "id": 1,
        "cmd": speed_left,
        "act": 3
    }

    ddsm_ser.write((json.dumps(command_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(command_left) + '\n').encode())
    time.sleep(0.1)

    current = vehicle.location.global_relative_frame
    dist = get_distance_meters(current, target_location)
    print(f"Distance to waypoint: {dist:.2f} m")
    if dist <= 1.0:
        print("Waypoint reached.")
        break


# Stop


# 17.397205, 78.489941
