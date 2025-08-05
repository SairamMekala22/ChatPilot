from dronekit import connect, VehicleMode, LocationGlobalRelative
from rplidar import RPLidar
import serial
import json
import time
import math
import sys

# ======== CONFIGURATION ========
CONNECTION_STRING = '/dev/ttyACM0'
LIDAR_PORT = '/dev/ttyUSB0'
DDSM_PORT = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200

TARGET_LAT = 17.397205
TARGET_LON = 78.489941
ALTITUDE = 0  # For ground vehicles

LIDAR_OBSTACLE_THRESHOLD = 700  # mm
FORWARD_SPEED = 30  # slow speed (out of 100) for LIDAR safety
TURN_SPEED = 40
SERVO_STOP_VALUE = 1500

# ======== GLOBALS ========
latest_servo1_value = None
latest_servo3_value = None

# ======== CONNECTIONS ========
print("[INFO] Connecting to vehicle...")
vehicle = connect(CONNECTION_STRING, baud=57600, wait_ready=True)

print("[INFO] Connecting to DDSM motor controller...")
ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE, timeout=1)

print("[INFO] Connecting to RPLIDAR...")
lidar = RPLidar(LIDAR_PORT)

# ======== HELPERS ========
def get_distance_meters(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    return int((servo_value - 1500) / 500 * 100)

def send_motor_speed(left, right):
    left = max(-100, min(100, left))
    right = max(-100, min(100, right))
    cmd_left = json.dumps({"T": 10010, "id": 2, "cmd": left, "act": 3}) + '\n'
    cmd_right = json.dumps({"T": 10010, "id": 1, "cmd": -right, "act": 3}) + '\n'
    ddsm_ser.write(cmd_right.encode())
    time.sleep(0.01)
    ddsm_ser.write(cmd_left.encode())

def stop_motors():
    send_motor_speed(0, 0)

def get_front_distance():
    try:
        scan = next(lidar.iter_scans(max_buf_meas=500))
        front = [d for (_, angle, d) in scan if angle <= 30 or angle >= 330]
        return min(front) if front else None
    except:
        return None

# ======== VEHICLE LISTENERS ========
@vehicle.on_message('SERVO_OUTPUT_RAW')
def servo_listener(self, name, message):
    global latest_servo1_value, latest_servo3_value
    latest_servo1_value = message.servo1_raw
    latest_servo3_value = message.servo3_raw

# ======== INITIALIZE VEHICLE ========
print("[INFO] Waiting for GPS fix...")
while not vehicle.location.global_frame.lat or vehicle.location.global_frame.lat == 0.0:
    time.sleep(1)

print("[INFO] Setting GUIDED mode...")
vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode.name != "GUIDED":
    time.sleep(1)

print("[INFO] Arming vehicle...")
vehicle.armed = True
while not vehicle.armed:
    time.sleep(1)

target_location = LocationGlobalRelative(TARGET_LAT, TARGET_LON, ALTITUDE)
print(f"[INFO] Navigating to: {TARGET_LAT}, {TARGET_LON}")
vehicle.simple_goto(target_location)

# ======== MAIN LOOP ========
try:
    while True:
        front_dist = get_front_distance()
        current = vehicle.location.global_relative_frame
        dist = get_distance_meters(current, target_location)

        print(f"[STATUS] Distance to target: {dist:.2f} m, LIDAR: {front_dist} mm")

        if dist <= 1.0:
            print("[SUCCESS] Reached target!")
            break

        if front_dist and front_dist < LIDAR_OBSTACLE_THRESHOLD:
            print("[AVOID] Obstacle detected! Executing avoidance...")
            stop_motors()
            time.sleep(0.5)
            send_motor_speed(-TURN_SPEED, TURN_SPEED)  # Turn in place
            time.sleep(0.7)
            stop_motors()
            continue

        # If clear path, move slowly toward target
        send_motor_speed(FORWARD_SPEED, FORWARD_SPEED)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("[ABORT] Interrupted by user. Stopping...")
finally:
    stop_motors()
    lidar.stop()
    lidar.disconnect()
    vehicle.close()
    ddsm_ser.close()
    print("[CLEANUP] Shutdown complete.")

