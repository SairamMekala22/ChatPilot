from dronekit import connect, VehicleMode, LocationGlobalRelative
from rplidar import RPLidar
import math
import time
import threading
import ddsm



# =================== CONSTANTS ===================
LIDAR_PORT = '/dev/ttyUSB0'     # Lidar port
PIXHAWK_PORT = '/dev/ttyACM0'   # Pixhawk serial port
BAUDRATE = 57600
MIN_DISTANCE = 500  # mm (obstacle avoidance threshold)
TARGET_LAT = 17.123456  # Replace with your target latitude
TARGET_LON = 78.123456  # Replace with your target longitude
ALTITUDE = 0.0          # For rovers, altitude is 0

latest_scan = None  # Global Lidar scan storage
latest_servo1_value = None
latest_servo3_value = None

# =================== FUNCTIONS ===================

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

def is_front_clear(scan_data):
    """Check if front sector (350° to 10°) is clear"""
    for (_, angle, dist) in scan_data:
        if (angle >= 350 or angle <= 10) and dist < MIN_DISTANCE and dist > 0:
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

def avoid_obstacle(vehicle, scan_data):
    """Perform an obstacle avoidance maneuver"""
    direction = decide_turn_direction(scan_data)
    print(f"[Avoidance] Turning {direction} to avoid obstacle...")

    if direction == 'left':
        vehicle.channels.overrides = {'1': 1425, '3': 1500}  # Turn left (steering channel 1)
    else:
        vehicle.channels.overrides = {'1': 1575, '3': 1500}  # Turn right

    time.sleep(1)  # Time to turn
    vehicle.channels.overrides = {'1': 1500, '3': 1500}  # Stop
    time.sleep(0.5)

def goto_position(vehicle, target_location):
    """Navigate to target GPS position with obstacle avoidance"""
    print(f"[Navigation] Moving to target: {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location)

    while True:
        dist_to_target = get_distance_meters(vehicle.location.global_relative_frame, target_location)
        print(f"[Navigation] Distance to target: {dist_to_target:.2f} meters")

        if dist_to_target <= 1.0:  # Arrived
            print("[Navigation] Target Reached!")
            break

        if latest_scan is not None and not is_front_clear(latest_scan):
            print("[Warning] Obstacle detected ahead!")
            vehicle.simple_goto(vehicle.location.global_relative_frame)  # halt forward
            avoid_obstacle(vehicle, latest_scan)  # perform avoidance
            vehicle.simple_goto(target_location)  # resume towards target

        time.sleep(1)


# =================== MAIN ===================

def main():
    global latest_scan

    print("[System] Connecting to Pixhawk...")
    vehicle = connect(PIXHAWK_PORT, baud=BAUDRATE, wait_ready=False)

    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw
        print(f"[SERVO] Servo1: {latest_servo1_value}, Servo3: {latest_servo3_value}")
        speed_right = scale_servo_to_speed(latest_servo1_value)
        speed_left = scale_servo_to_speed(latest_servo3_value)
        print(f"speed left : {speed_left}, speed right : {speed_right}")
        ddsm.writeRight(speed_right)
        time.sleep(0.01)
        ddsm.writeLeft(speed_left)
        time.sleep(0.1)

    # print("[System] Setting GUIDED mode...")
    # vehicle.mode = VehicleMode("GUIDED")
    # while vehicle.mode.name != "GUIDED":
    #     print("[System] Waiting for GUIDED mode...")
    #     time.sleep(1)

    # print("[System] Arming vehicle...")
    # vehicle.armed = True
    # while not vehicle.armed:
    #     print("[System] Waiting for arming...")
    #     time.sleep(1)

    print("[System] Starting LIDAR...")
    lidar = RPLidar(LIDAR_PORT)
    threading.Thread(target=lidar_thread_func, args=(lidar,), daemon=True).start()

    target_location = LocationGlobalRelative(TARGET_LAT, TARGET_LON, ALTITUDE)

    # try:
    #     goto_position(vehicle, target_location)
    # except KeyboardInterrupt:
    #     print("[System] User interrupted. Exiting...")
    # finally:
    #     print("[System] Cleaning up...")
    #     vehicle.channels.overrides = {}
    #     vehicle.armed = False
    #     vehicle.close()
    #     lidar.stop()
    #     lidar.stop_motor()
    #     lidar.disconnect()
    #     print("[System] Shutdown complete.")
    try:
        while True:
            if latest_scan is None:
                print("Waiting for LIDAR data...")
                time.sleep(1)
                continue
            
            # Throttle 2, steering 1

            # Obstacle check
            if not is_front_clear(latest_scan):
                print("[Obstacle] Avoiding...")
                vehicle.channels.overrides = {'1': 1550, '2': 1500}  # Turn right
                time.sleep(0.2)
                #avoid_obstacle(vehicle, lastest_scan)
                continue
                #vehicle.channels.overrides = {'1': 1500, '2': 1500}  # Stop briefly
                #time.sleep(0.5)

            vehicle.channels.overrides = {'1': 1500, '2': 1600}
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping test...")

    finally:
        vehicle.channels.overrides = {}
        vehicle.armed = False
        vehicle.close()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        ddsm.ddsm_ser.close()

if __name__ == "__main__":
    main()
