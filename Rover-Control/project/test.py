from rplidar import RPLidar
import time
import requests

PORT_NAME = '/dev/ttyUSB0'
MIN_DISTANCE = 500  # mm
SCAN_COUNT = 25

ROVER_IP = '192.168.4.1'  # Update with your rover IP

class RoverControl:
    def __init__(self, ip_address):
        self.ip = ip_address

    def send_command(self, motor, cmd, act):
        url = f'http://{self.ip}/js'
        payload = {"T":10010,"id":motor,"cmd":cmd,"act":act}
        params = {"json": str(payload).replace("'",'"')}

        try:
            response = requests.get(url,params = params)
            print(f'Sent command to motor {motor}: cmd={cmd}, act={act}, response={response.status_code}')
        except Exception as e:
            print(f'Failed to send command: {e}')

    def move_forward(self, speed=10):
        self.send_command(1, speed, 3)
        self.send_command(2, -speed, 3)

    def move_backward(self, speed=10):
        self.send_command(1, -speed, 3)
        self.send_command(2, speed, 3)

    def turn_left(self, speed=10):
        self.send_command(1, -speed, 3)
        self.send_command(2, -speed, 3)

    def turn_right(self, speed=10):
        self.send_command(1, speed, 3)
        self.send_command(2, speed, 3)

    def stop(self):
        self.send_command(1, 0, 3)
        self.send_command(2, 0, 3)

def process_scan(scan):
    front_clear = True
    left_clear = True
    right_clear = True

    for (_, angle, distance) in scan:
        if distance == 0:
            continue
        if (angle >= 350 or angle <= 10) and distance < MIN_DISTANCE:
            front_clear = False
        elif 60 <= angle <= 120 and distance < MIN_DISTANCE:
            right_clear = False
        elif 240 <= angle <= 300 and distance < MIN_DISTANCE:
            left_clear = False

    print(f"Front clear: {front_clear}, Left clear: {left_clear}, Right clear: {right_clear}")
    return front_clear, left_clear, right_clear

lidar = RPLidar(PORT_NAME,baudrate = 115200)
lidar.stop_motor()
lidar.stop()
lidar.disconnect()
lidar = RPLidar(PORT_NAME)
rover = RoverControl(ROVER_IP)

try:
    print("Starting autonomous driving...")
    rover.stop()
    time.sleep(1)

    current_action = None
    lidar.stop()
    lidar.stop_motor()
    time.sleep(1)
    lidar.start_motor()
    time.sleep(1)
    for i, scan in enumerate(lidar.iter_scans()):
        print(f'Scan {i}: {len(scan)} measurements')

        front_clear, left_clear, right_clear = process_scan(scan)

        # Decision Logic
        if front_clear and current_action != 'forward':
            rover.move_forward(speed=20)
            current_action = 'forward'
            print("Moving forward...")

        elif not front_clear and left_clear and current_action != 'left':
            rover.turn_left(speed=20)
            current_action = 'left'
            print("Turning left...")

        elif not front_clear and right_clear and current_action != 'right':
            rover.turn_right(speed=20)
            current_action = 'right'
            print("Turning right...")

        elif not front_clear and not left_clear and not right_clear and current_action != 'reverse':
            rover.move_backward(speed=20)
            current_action = 'reverse'
            print("Reversing...")

        if i > SCAN_COUNT:
            print("Completed scan cycles. Stopping...")
            #lidar._serial.flushInput()
            #break
except Exception:
    lidar.stop()
    lidar.disconnect()
    

finally:
    print("Stopping LIDAR and Rover...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    rover.stop()

