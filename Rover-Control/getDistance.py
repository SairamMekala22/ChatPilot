from dronekit import connect, VehicleMode
import time
import os
import math

def get_haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    print(R * c)

    return R * c

coordinates = []

def read_coordinates_from_file(filename):
    # coordinates = []

    try:
        with open(filename, 'r') as file:
            print(file)
            for line in file:
                line = line.replace(" ", "")
                parts = line.strip().split(',')
                if True:
                    try:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        coordinates.append((lat, lon))
                    except ValueError:
                        print(f"Skipping invalid line: {line.strip()}")
    except FileNotFoundError:
        print(f"File not found: {filename}")
    
    #coordinates.reverse()
    return coordinates

for i in range(1, len(coordinates)):
    wp1 = coordinates[i - 1]
    wp2 = coordinates[i]
    get_haversine_distance(wp1[0], wp1[1], wp2[0], wp2[1])