from dronekit import connect
import time
import serial
import math
import json
import threading
from rplidar import RPLidar


LIDAR_PORT = '/dev/ttyUSB0'
DDSM_PORT = '/dev/ttyACM0'
VEHICLE_CONNECTIION = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200
OBSTACLE_THRESHOLD = 1000  # in mm
HEADING_TOLERANCE = 10  # degrees
WAYPOINT_REACHED_RADIUS = 0.5

TURN_SPEED = 40
FORWARD_SPEED = 80

path = []
'''
 path = [
         (17.385044, 78.486671),
         (17.385144, 78.486771),
         (17.385244, 78.486871)
 ]
'''
arrived = False
stop = False
latest_scan = None
ddsm_ser = None
vehicle = None

def read_coordinates_from_file(filename):
    coordinates = []

    try:
        with open(filename, 'r') as file:
            print(file)
            for line in file:
                #line = line.trim()
                line = line.replace(" ", "")
                print(line)
                parts = line.strip().split(',')
                print(parts)
                if True:
                    try:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        # alt = 0.0
                        coordinates.append((lat, lon))
                    except ValueError:
                        print(f"Skipping invalid line: {line.strip()}")
    except FileNotFoundError:
        print(f"File not found: {filename}")
    
    #coordinates.reverse()
    return coordinates

def get_location():
    # global vehicle
    current = vehicle.location.global_relative_frame
    return current.lat, current.lon

def get_heading():
    # global vehicle
    # heading = vehicle.attitude.yaw
    # heading_degrees = rad2deg(heading)
    return vehicle.heading

def input_listener():
    global arrived, stop
    while True:
        user_input = input("Press 'm' to confirm it reached waypoint")
        if user_input == "m":
            arrived = True
        elif user_input == "s":
            stop =True

def get_haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

def get_desired_heading(current_lat, current_lon, target_lat, target_lon):
    """
    Calculate the desired heading (bearing) from current location to target location.
    
    Args:
        current_heading (float): Current yaw/heading of the vehicle (0–360 degrees).
        current_location (tuple): (lat, lon) in degrees.
        target_location (tuple): (lat, lon) in degrees.
    
    Returns:
        float: Desired heading in degrees (0–360) to face the target.
    """
    lat1, lon1 = current_lat, current_lon
    lat2, lon2 = target_lat, target_lon
    # lat1, lon1 = 17.3973503, 78.4900178
    # lat2, lon2 = 17.3973573, 78.4899868
    # lat1, lon1 = map(math.radians, current_location)
    # lat2, lon2 = map(math.radians, target_location)

    delta_lon = lon2 - lon1

    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(delta_lon)

    initial_bearing = math.atan2(x, y)
    initial_bearing_deg = math.degrees(initial_bearing)
    desired_heading = (initial_bearing_deg + 360) % 360
    print(desired_heading)
    #time.sleep(20)
    return int(desired_heading)

def get_heading_error(current_heading, desired_heading):
    error = (desired_heading - current_heading + 540) % 360 - 180
    return error

def lidar_thread_func(lidar):
    global latest_scan
    for scan in lidar.iter_scans():
        latest_scan = scan

def is_front_clear():
    # global latest_scan
    while latest_scan is None:
        return True     # assume clear
    scan_data = latest_scan
    for (_, angle, dist) in scan_data:
        if (angle >= 345 and angle <= 15) and dist < OBSTACLE_THRESHOLD and dist > 0:
            return False
    return True

def is_left_clear():
    # global latest_scan
    scan_data = latest_scan
    for (_, angle, dist) in scan_data:
        if (angle >= 310 and angle <= 340) and dist < OBSTACLE_THRESHOLD and dist > 0:
            return False
    return True

def set_motor_speeds(left, right):
    global ddsm_ser
    command_right = {
        "T": 10010,
        "id": 1,
        "cmd": left,  # reverse polarity for right wheel
        "act": 3
    }
    command_left = {
        "T": 10010,
        "id": 2,
        "cmd": -right,
        "act": 3
    }
    ddsm_ser.write((json.dumps(command_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(command_left) + '\n').encode())

def main():
    global vehicle, ddsm_ser
    coord_file = "cords.txt"
    path = read_coordinates_from_file(coord_file)
    print(path)
    if not path:
        print("No coordinates loaded from file.")
        return
    print("Coordinates read success...")
    #threading.Thread(target=input_listener, daemon=True).start()
    lidar = RPLidar(LIDAR_PORT)
    threading.Thread(target=lidar_thread_func, args=(lidar,), daemon=True).start()
    while True:
        if latest_scan is None:
            print("Waiting for LIDAR data...")
            time.sleep(1)
            continue 
        else:
            print("Lidar started...")
            break
    
    vehicle = connect(VEHICLE_CONNECTIION, wait_ready=True, baud=57600)
    print("vehicle connected")
    ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)
    print("DDSM Connected")

    try:
        obj = Bug1Navigator(path)
        obj.run()
    except KeyboardInterrupt:
        print("Stopping test...")
        set_motor_speeds(0,0)
    finally:
        vehicle.close()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        ddsm_ser.close()

class Bug1Navigator:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_wp_index = 0
        self.state = "GO_TO_GOAL"
        self.hit_point = None
        self.leave_point = None
        self.closest_point = None
        self.min_distance_to_goal = float('inf')

    def go_to_goal(self):
        current_lat, current_lon = get_location()
        target_lat, target_lon = self.waypoints[self.current_wp_index]
        distance = get_haversine_distance(current_lat, current_lon, target_lat, target_lon)
        #print(type(distance))
        #time.sleep(10)

        if distance < WAYPOINT_REACHED_RADIUS:
            print(f"Waypoint {self.current_wp_index + 1} reached.")
            self.current_wp_index += 1
            #arrived = False
            return

        desired_heading = get_desired_heading(current_lat, current_lon, target_lat, target_lon)
        current_heading = get_heading()
        error = get_heading_error(current_heading, desired_heading)
        print(f"desired : {desired_heading} current : {current_heading} error : {error}")

        if abs(error) > HEADING_TOLERANCE:
            if error > 0:
                set_motor_speeds(TURN_SPEED, -TURN_SPEED)
            else:
                set_motor_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED)

        if not is_front_clear():
            self.state = "FOLLOW_OBSTACLE"
            self.hit_point = (current_lat, current_lon)
            self.closest_point = self.hit_point
            self.min_distance_to_goal = distance
            print("Obstacle encountered, switching to FOLLOW_OBSTACLE.")

    def follow_obstacle(self):
        if not is_front_clear():
            set_motor_speeds(TURN_SPEED, -TURN_SPEED)
        elif not is_left_clear():
            set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED)
        else:
            self.state = "GO_TO_GOAL"
            # set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED - 10)

    def run(self):
        print("Running.......")
        while self.current_wp_index < len(self.waypoints):
            print(f"[State] --> {self.state}")
            if stop:
                print("Emergency Stop")
                set_motor_speeds(0,0)
                break
            if self.state == "GO_TO_GOAL":
                self.go_to_goal()
            elif self.state == "FOLLOW_OBSTACLE":
                self.follow_obstacle()
            time.sleep(0.1)

        print("All waypoints reached. Stopping rover.")
        set_motor_speeds(0, 0)

main()
