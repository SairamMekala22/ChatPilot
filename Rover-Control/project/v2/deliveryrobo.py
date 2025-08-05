import serial
import argparse
import threading
from dronekit import connect, VehicleMode
import json
import time
# from rplidar import RPLidar

# Global variables to store the latest sensor data
latest_servo1_value = None
latest_servo3_value = None

def read_ddsm_serial(ddsm_ser):
    """Thread function to read and optionally log DDSM serial data."""
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
            if data:
                pass  # Optionally uncomment to print: print(f"[DDSM] {data}", end='')
        except Exception as e:
            print("error")

def input_listener(vehicle):
    while True:
        user_input = input("Press 'Y' to log current GPS coordinates: ").strip().upper()
        if user_input == "y":
            #global vehicle
            location = vehicle.location.global_frame
            lat = location.lat
            lon = location.lon
            with open("/home/kmit/project/v2/logged_coordinates.txt", "a") as f:
                f.write(f"{lat}, {lon}, {alt}\n")
            print(f"[GPS Saved] {lat}, {lon}, {alt}")

def scale_servo_to_speed(servo_value):
    """Maps servo PWM range to speed values, with None handling."""
    if servo_value is None:
        return 0
    # Maps servo_value (1000-2000 µs) to -100 to 100, centered around 1735
    return int((servo_value - 1500) / 500 * 100)

def main():

    # Connect to DDSM serial
    ddsm_ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)

    # Start thread to read from DDSM
    # threading.Thread(target=read_ddsm_serial, args=(ddsm_ser,), daemon=True).start()

    # Connect to Pixhawk
    print("[Info] Connecting to Pixhawk...")
    vehicle = connect('/dev/ttyACM1', wait_ready=True, baud=57600)
    print("[Info] Connected to Pixhawk.")

    # collect cordinates
    threading.Thread(target=input_listener, args=(vehicle,), daemon=True).start()
    @vehicle.on_message('GPS_RAW_INT')
    def gps_callback(self,name,message):
         global vehicle
         location = vehicle.location.global_frame
         lat = location.lat
         lon = location.lon
         alt = location.alt
         with open("/home/kmit/project/v2/logged_coordinates.txt", "a") as f:
             f.write(f"{lat}, {lon}, {alt}\n")
         print(f"[GPS Saved] {lat}, {lon}, {alt}")
    # Define the servo output listener
    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw  # Right-side wheels
        latest_servo3_value = message.servo3_raw  # Left-side wheels
        print(f"[Servo Output] Channel 1: {latest_servo1_value} µs, Channel 3: {latest_servo3_value} µs")



    try:
        while True:
            # Retrieve latest servo values
            servo1 = latest_servo1_value
            servo3 = latest_servo3_value

            # Convert servo values to wheel speeds (scaled to -100 to 100 range)
            speed_right = scale_servo_to_speed(servo1)
            speed_left = scale_servo_to_speed(servo3)
            speed_right = max(-100, min(100, speed_right))  # Limit speed to range -100 to 100
            speed_left = max(-100, min(100, speed_left))    # Limit speed to range -100 to 100
            
            command_right = {
                "T": 10010,
                "id": 1,  # Right-side wheels
                "cmd": speed_right,  # Send speed as negative for right motor
                "act": 3
            }
            command_left = {
                "T": 10010,
                "id": 2,  # Left-side wheels
                "cmd": -speed_left,  # Send speed as positive for left motor
                "act": 3
            }

            # Send commands to DDSM
            ddsm_ser.write((json.dumps(command_right) + '\n').encode())
            time.sleep(0.01)  # Small delay between commands
            ddsm_ser.write((json.dumps(command_left) + '\n').encode())
            print(f"[Sent to DDSM] Right: {command_right}")
            print(f"[Sent to DDSM] Left: {command_left}")
            time.sleep(0.1)  # Maintain 10Hz loop

    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()

if __name__ == "__main__":
    main()
