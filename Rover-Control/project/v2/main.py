from dronekit import connect, VehicleMode, LocationGlobalRelative
from rplidar import RPLidar
import math
import time
import threading
import serial
import json

# =================== CONSTANTS ===================
LIDAR_PORT = '/dev/ttyUSB0'     # Lidar port
PIXHAWK_PORT = '/dev/ttyACM1'   # Pixhawk serial port
DDSM_PORT = '/dev/ttyACM0'
SERIAL_BAUDRATE = 115200
BAUDRATE = 57600
MIN_DISTANCE = 500  # mm (obstacle avoidance threshold)
TARGET_LAT = 17.3973234  # Replace with your target latitude
TARGET_LON = 78.4899548 # Replace with your target longitude
ALTITUDE = 0.0          # For rovers, altitude is 0

TURN_SPEED = 30
FORWARD_SPEED = 40

arrived = False
latest_scan = None  # Global Lidar scan storage
latest_servo1_value = None
latest_servo3_value = None

ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)
path = []
'''
 path = [
         (17.385044, 78.486671),
         (17.385144, 78.486771),
         (17.385244, 78.486871)
 ]
'''
# =================== FUNCTIONS ===================

def input_listener():
    global arrived
    while True:
        user_input = input("Press 'm' to confirm it reached waypoint")
        if user_input == "m":
            arrived = True

def read_coordinates_from_file(filename):
    coordinates = []

    try:
        with open(filename, 'r') as file:
            for line in file:
                parts = line.strip().split(',')
                if len(parts) == 3:
                    try:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        alt = ALTITUDE
                        coordinates.append((lat, lon, alt))
                    except ValueError:
                        print(f"Skipping invalid line: {line.strip()}")
    except FileNotFoundError:
        print(f"File not found: {filename}")

    return coordinates

def get_distance_meters(aLocation1, aLocation2):
    """Calculate distance between two GPS coordinates (in meters)"""
    dlat = aLocation2.lat - aLocation1.lat
    dlon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

def lidar_thread_func(lidar):
    """Lidar scanning in a background thread"""
    global latest_scan
    for scan in lidar.iter_scans():
        latest_scan = scan

def is_front_clear():
    global latest_scan
    scan_data = latest_scan
    for (_, angle, dist) in scan_data:
        if (angle >= 350 or angle <= 10) and dist < MIN_DISTANCE and dist > 0:
            return False
    return True

def is_left_clear():
    global latest_scan
    scan_data = latest_scan
    for (_, angle, dist) in scan_data:
        if (angle >= 270 or angle <= 340) and dist < MIN_DISTANCE+100 and dist > 0:
            return False
    return True

def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    # Map servo PWM (1000-2000 µs) to speed (-100 to 100)
    return int((servo_value - 1500) / 500 * 100)

def get_sector_distance(scan_data, min_angle, max_angle):
    """Get average distance in a sector"""
    distances = [dist for (_, angle, dist) in scan_data
                 if min_angle <= angle <= max_angle and dist > 0]
    if len(distances) == 0:
        return float('inf')
    return sum(distances) / len(distances)

def decide_turn_direction(scan_data):
    """Decide whether to turn left or right based on clearance"""
    left_clearance = get_sector_distance(scan_data, 60, 120)    # left sector
    right_clearance = get_sector_distance(scan_data, 240, 300)  # right sector
    print(f"[Decision] Left clearance: {left_clearance:.2f} mm, Right clearance: {right_clearance:.2f} mm")
    return 'left' if left_clearance > right_clearance else 'right'

def avoid_obstacle():   
    """Perform an obstacle avoidance maneuver"""
    motor_control(0,0) # stop 
    time.sleep(0.3)
    while not is_front_clear():
        print("[Obstacle] Avoiding...")
        motor_control(20,-20) # turn right
        time.sleep(0.5)

    motor_control(20 , 20) # move forward
    time.sleep(1.5)

def follow_obstacle():
    while True:
        if not is_front_clear():
            motor_control(TURN_SPEED, -TURN_SPEED)
            print("AVOID -> turn right")
        elif not is_left_clear():
            motor_control(FORWARD_SPEED, FORWARD_SPEED)
            print("AVOID -> move forward")
        else:
            break

def goto_position(vehicle, target_location):
    global latest_scan, latest_servo1_value, latest_servo3_value, arrived
    print(f"[Navigation] Moving to target: {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location)

    while True:
        dist_to_target = get_distance_meters(vehicle.location.global_relative_frame, target_location)
        print(f"[Navigation] Distance to target: {dist_to_target:.2f} meters")

        if dist_to_target <= 1.0 or arrived:  # Arrived
            print("[Navigation] Target Reached!")
            arrived=False
            break

        if latest_scan is not None and not is_front_clear():
            print("[Warning] Obstacle detected ahead!")
            #avoid_obstacle()  # perform avoidance
            follow_obstacle()

        servo1 = latest_servo1_value
        servo3 = latest_servo3_value
        speed_left = scale_servo_to_speed(servo1)
        speed_right = scale_servo_to_speed(servo3)
        motor_control(speed_left, speed_right)
        time.sleep(0.1)

def motor_control(left, right):
    global ddsm_ser
    command_right = {
        "T": 10010,
        "id": 2,
        "cmd": -right,  # reverse polarity for right wheel
        "act": 3
    }
    command_left = {
        "T": 10010,
        "id": 1,
        "cmd": left,
        "act": 3
    }
    ddsm_ser.write((json.dumps(command_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(command_left) + '\n').encode())

# =================== MAIN ===================

def main():
    global latest_scan, path
    coord_file = "bb.txt"
    path = read_coordinates_from_file(coord_file)

    print("[System] Connecting to Pixhawk...")
    vehicle = connect(PIXHAWK_PORT, baud=BAUDRATE, wait_ready=False)

    threading.Thread(target=input_listener, daemon=True).start()

    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw
        print(f"[SERVO] Servo1: {latest_servo1_value}, Servo3: {latest_servo3_value}")
          
    print("[System] Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("[System] Waiting for GUIDED mode...")
        time.sleep(1)

    print("[System] Arming vehicle...")
    vehicle.armed = True
    while not vehicle.armed:
        print("[System] Waiting for arming...")
        time.sleep(1)

    print("[System] Starting LIDAR...")
    lidar = RPLidar(LIDAR_PORT)
    threading.Thread(target=lidar_thread_func, args=(lidar,), daemon=True).start()
    
    try:
        for x in path:
            target_location = LocationGlobalRelative(x[0], x[1], ALTITUDE)
            goto_position(vehicle, target_location)

    except KeyboardInterrupt:
        print("Stopping test...")
        motor_control(0,0)

    finally:
        vehicle.channels.overrides = {}
        vehicle.armed = False
        vehicle.close()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        ddsm_ser.close()

main()
