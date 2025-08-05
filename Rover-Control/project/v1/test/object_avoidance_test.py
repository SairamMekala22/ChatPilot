# if front_clear and !left_clear ----> move forward
# if !front_clear and !left_clear ----> turn right
# if !front_clear and left_clear ----> trun right
# if front_clear and left_clear ----> move to location 
import time
from rplidar import RPLidar
import threading
import serial
import json

# Constants
LIDAR_PORT = '/dev/ttyUSB0'
DDSM_PORT = '/dev/ttyACM0' # Change as needed
FRONT_ANGLE_RANGE = 30  # +/- degrees around front
SIDE_ANGLE_LEFT = 90
SIDE_ANGLE_RIGHT = 270
MIN_DISTANCE_FRONT = 500

TURN_SPEED = 30
FORWARD_SPEED = 40

# Init LIDAR
lidar = RPLidar(LIDAR_PORT)
state = "forward"
latest_scan = None
ddsm_ser = serial.Serial(DDSM_PORT, baudrate=115200)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)

def lidar_thread_func(lidar):
    global latest_scan
    for scan in lidar.iter_scans():
        latest_scan = scan

def is_front_clear():
    global latest_scan
    scan_data = latest_scan
    for (_, angle, dist) in scan_data:
        if (angle >= 345 or angle <= 15) and dist < MIN_DISTANCE_FRONT and dist > 0:
            return False
    return True

def is_left_clear():
    global latest_scan
    scan_data = latest_scan
    for (_, angle, dist) in scan_data:
        if (angle >= 270 or angle <= 340) and dist < MIN_DISTANCE_FRONT+100 and dist > 0:
            return False
    return True

def motor_control(left, right):
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


def move_forward(): 
    global state
    motor_control(30,30)
    if not is_front_clear():
        state =  "avoid"
        return 
    time.sleep(0.5)

def follow_obstacle():
    if not is_front_clear():
        motor_control(TURN_SPEED, -TURN_SPEED)
        print("AVOID -> turn right")
    elif not is_left_clear():
        motor_control(FORWARD_SPEED, FORWARD_SPEED)
        print("AVOID -> turn forward")
    else:
        motor_control(-TURN_SPEED, TURN_SPEED)   
        print("AVOID -> turn left")
        # motor_control(FORWARD_SPEED, FORWARD_SPEED - 10)
       

def main():
    global latest_scan
    lidar = RPLidar(LIDAR_PORT)
    threading.Thread(target=lidar_thread_func, args=(lidar,), daemon=True).start()
    try:
        while True:
            if latest_scan is None:
                print("Waiting for LIDAR data...")
                time.sleep(1)
                continue  
              
            if state == "forward":
                print("STATE -> FORWARD")
                move_forward()
            elif state == "avoid":
                print("STATE -> AVOID")
                follow_obstacle()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping test...")
        motor_control(0, 0)
        time.sleep(0.5)

    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        ddsm_ser.close()

if __name__ == "__main__":
    main()

