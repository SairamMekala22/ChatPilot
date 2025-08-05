from dronekit import connect, VehicleMode
import time

# Connect to the vehicle (replace with your connection string)
#vehicle = connect('127.0.0.1:14550', wait_ready=True)
vehicle = connect('/dev/ttyACM1', wait_ready=True, baud=57600)

# Function to get current location
def get_location():
    loc = vehicle.location.global_frame
    return (loc.lat, loc.lon, loc.alt)

# Store location every 5 seconds
try:
    while True:
        location = get_location()
        lat = location[0]
        lon = location[1]
        alt = location[2]
        with open("/home/kmit/project/v2/logged_coordinates1.txt", "a") as f:
            f.write(f"{lat}, {lon}, {alt}\n")
        print(f"[GPS Saved] {lat}, {lon}, {alt}")
        #print(f"Latitude: {location[0]}, Longitude: {location[1]}, Altitude: {location[2]}")
        # You can store this to a file or database instead
        time.sleep(5)
except KeyboardInterrupt:
    print("Location logging stopped by user.")

# Close vehicle connection
vehicle.close()

