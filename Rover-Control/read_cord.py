from dronekit import connect, VehicleMode
import time
import os

# Connect to the vehicle (replace with your connection string)
# vehicle = connect('127.0.0.1:14551', wait_ready=False)
vehicle = connect('127.0.0.1:14550', wait_ready=False)
print("connected")
# vehicle = connect('u', wait_ready=True, baud=57600)

# Function to get current location
def get_location():
    loc = vehicle.location.global_frame
    return (loc.lat, loc.lon, loc.alt)

# Store location every 5 seconds
try:
    while True:
        user_input = input("Enter 'r' to write to file (or 'q' to quit): ").strip().lower()
    
        if user_input == 'r':
            location = get_location()
            lat = location[0]
            lon = location[1]
            alt = location[2]
            file_path = os.path.abspath("cords.txt")
            with open(file_path, "a") as file:
                file.write(f"{lat}, {lon}, {alt}\n")
            print(f"[GPS Saved] {lat}, {lon}, {alt}")
            print(f"Wrote to:\n{file_path}\n")
        elif user_input == 'q':
            print("Exiting program.")
            break
except KeyboardInterrupt:
    print("Location logging stopped by user.")

# Close vehicle connection
vehicle.close()

