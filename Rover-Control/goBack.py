from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# Connect to the vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)
print("Connected to vehicle")

# Read waypoints from file
def read_waypoints(file_path="cords.txt"):
    waypoints = []
    with open(file_path, 'r') as file:
        for line in file:
            lat, lon, alt = map(float, line.strip().split(','))
            waypoints.append(LocationGlobalRelative(lat, lon, alt))
    return waypoints

# Calculate distance between two GPS coordinates
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlong * 1.113195e5)**2)

# Go to a specific waypoint
def goto_waypoint(location, threshold=1.0):
    print(f"Going to waypoint: {location.lat}, {location.lon}")
    vehicle.simple_goto(location)

    while True:
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, location)
        print(f"Distance to waypoint: {distance:.2f} m")
        if distance < threshold:
            print("Reached waypoint\n")
            break
        time.sleep(2)

# Main navigation routine
waypoints = read_waypoints()

if len(waypoints) < 2:
    print("Not enough waypoints to navigate.")
else:
    print(f"Starting navigation through {len(waypoints)} waypoints...\n")

    # Arm the vehicle and set to GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    # Move through all waypoints in order
    for wp in waypoints:
        goto_waypoint(wp)

    # Finally return to the first waypoint
    print("Returning to start...")
    goto_waypoint(waypoints[0])

    print("Navigation complete.")

# Close vehicle connection
vehicle.close()
