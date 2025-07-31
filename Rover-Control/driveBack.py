from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import time
import serial
import json

LIDAR_PORT = '/dev/ttyUSB0'     # Lidar port
PIXHAWK_PORT = '/dev/ttyACM1'   # Pixhawk serial port
DDSM_PORT = '/dev/ttyACM0'
SERIAL_BAUDRATE = 115200
BAUDRATE = 57600
MIN_DISTANCE = 500 
TARGET_LAT = 17.3973234  
TARGET_LON = 78.4899548
ALTITUDE = 0.0    
WAYPOINT_REACHED_RADIUS = 1      
FILE_NAME = "cords.txt"
TURN_SPEED = 30
FORWARD_SPEED = 40



def getCoordinates(fileName):
    coordinates = []
    with open(fileName, 'r') as file:
        for line in file:
            lat, lon, corr = map(float, line.strip().split(","))
            coordinates.append(LocationGlobalRelative(lat, lon, ALTITUDE))
    
    return coordinates


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

def goto_location(vehicle, targetLocation):
    vehicle.simple_goto(targetLocation)
    while True:
        current_location = vehicle.location.global_relative_frame
        dist_to_target = get_haversine_distance(current_location.lat, current_location.lon , targetLocation.lat, targetLocation.lon)
        print(f"[Navigation] Distance to target: {dist_to_target:.2f} meters")

        if dist_to_target <= WAYPOINT_REACHED_RADIUS : # Arrived
            print("[Navigation] Target Reached!")
            break

if (__name__ == "__main__"):
    global path
    path = getCoordinates(FILE_NAME)
    # print(path)

    print("Connecting to vehicle....")
    # vehicle = connect(PIXHAWK_PORT, baud = BAUDRATE, wait_ready = False)
    vehicle = connect("127.0.0.1:14550", baud = BAUDRATE, wait_ready = False)
    print("Connection success")

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print("Armed successfully...")

    vehicle.mode = VehicleMode("Guided")
    while (vehicle.mode.name != "Guided"):
        print("Waiting for guided mode...")
        time.sleep(1)
    print("Vehicle mode -> {vehicle.mode}")