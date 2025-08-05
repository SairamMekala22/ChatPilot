import serial
import json
import threading
import time
from dronekit import connect, VehicleMode
from rplidar import RPLidar

# ======== CONSTANTS =========
LIDAR_PORT = '/dev/ttyUSB0'
DDSM_PORT = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200
MIN_DISTANCE = 500  # mm obstacle threshold


# ======== GLOBALS =========
latest_scan = None
latest_servo1_value = None
latest_servo3_value = None

ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)

# ======== LIDAR THREAD =========
def lidar_thread_func(lidar):
    global latest_scan
    for scan in lidar.iter_scans():
        latest_scan = scan

def is_front_clear(scan_data):
    for (_, angle, dist) in scan_data:
        if (angle >= 350 or angle <= 10) and dist < MIN_DISTANCE and dist > 0:
            return False
    return True

# ======== SERVO TO SPEED MAPPING =========
def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    # Map servo PWM (1000-2000 µs) to speed (-100 to 100)
    return int((servo_value - 1500) / 500 * 100)

def motor_control(left, right):
    global ddsm_ser
    command_right = {
        "T": 10010,
        "id": 1,
        "cmd": right,  # reverse polarity for right wheel
        "act": 3
    }
    command_left = {
        "T": 10010,
        "id": 2,
        "cmd": -left,
        "act": 3
    }
    ddsm_ser.write((json.dumps(command_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(command_left) + '\n').encode())

# ======== MAIN PROGRAM =========
def main():
    global latest_servo1_value, latest_servo3_value, latest_scan

    # Connect to Pixhawk
    vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=False)
    
    # Servo Output Listener
    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw
        print(f"[SERVO] Servo1: {latest_servo1_value}, Servo3: {latest_servo3_value}")

    # Start LIDAR thread
    lidar = RPLidar(LIDAR_PORT)
    threading.Thread(target=lidar_thread_func, args=(lidar,), daemon=True).start()

    try:
        while True:
            if latest_scan is None:
                print("Waiting for LIDAR data...")
                time.sleep(1)
                continue  

            # Obstacle check
            if not is_front_clear(latest_scan):
                print("Obstacle Avoiding...")
                vehicle.channels.overrides = {'1': 1550, '2': 1500}  # Turn right
                time.sleep(0.1)
            else:
                print("Moving Forward...")
                vehicle.channels.overrides = {'1': 1500, '2': 1600}

            # Read latest servo outputs
            servo1 = latest_servo1_value
            servo3 = latest_servo3_value
            # Convert PWM to speed (-100 to 100)
            speed_right = scale_servo_to_speed(servo1)
            speed_left = scale_servo_to_speed(servo3)
            speed_right = max(-100, min(100, speed_right))
            speed_left = max(-100, min(100, speed_left))
            print(f"Speed left: {speed_left}, right: {speed_right}")
            motor_control(speed_left, speed_right)

            time.sleep(0.5) # scan for obstacle for every 0.5 sec

    except KeyboardInterrupt:
        print("Stopping test...")
        motor_control(0, 0)

    finally:
        vehicle.channels.overrides = {}
        # vehicle.armed = False
        vehicle.close()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        ddsm_ser.close()

if __name__ == "__main__":
    main()
