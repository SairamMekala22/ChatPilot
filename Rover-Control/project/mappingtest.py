import time
import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar
import requests

PORT_NAME = '/dev/ttyUSB0'
MIN_DISTANCE = 500  # mm
SCAN_COUNT = 25
ROVER_IP = '192.168.4.1'  # Update with your rover IP

# Rover Control Class for sending commands
class RoverControl:
    def __init__(self, ip_address):
        self.ip = ip_address

    def send_command(self, motor, cmd, act):
        url = f'http://{self.ip}/js'
        payload = {"T": 10010, "id": motor, "cmd": cmd, "act": act}
        params = {"json": str(payload).replace("'", '"')}

        try:
            response = requests.get(url, params=params)
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


# Handle lidar data processing and decision making
def process_scan(scan):
    front_clear = True
    left_clear = True
    right_clear = True

    for scan_data in scan:
        try:
            _, angle, distance = scan_data  # Unpack scan data
            if distance == 0:
                continue
            if (angle >= 350 or angle <= 10) and distance < MIN_DISTANCE:
                front_clear = False
            elif 60 <= angle <= 120 and distance < MIN_DISTANCE:
                right_clear = False
            elif 240 <= angle <= 300 and distance < MIN_DISTANCE:
                left_clear = False
        except ValueError:
            print(f"Error unpacking scan data: {scan_data}")
            continue  # Skip invalid data

    print(f"Front clear: {front_clear}, Left clear: {left_clear}, Right clear: {right_clear}")
    return front_clear, left_clear, right_clear


# Handle restarting the lidar
def restart_lidar(lidar):
    """Restarts the Lidar if an error occurs."""
    try:
        print("Restarting Lidar...")
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()
        time.sleep(1)  # Give some time for the Lidar to reset

        # Clear the buffer
        lidar._serial.flushInput()
        time.sleep(1)  # Ensure the buffer is cleared

        lidar = RPLidar(PORT_NAME, baudrate=115200)  # Reinitialize
        lidar.start_motor()
        print("Lidar restarted successfully.")
    except Exception as e:
        print(f"Failed to restart Lidar: {e}")


# Update map with new Lidar scan data
def update_map(scan, current_pos, ax):
    """Update the map with new Lidar scan data."""
    for _, angle, distance in scan:
        if distance == 0:
            continue

        # Convert polar coordinates (angle, distance) to Cartesian (x, y)
        x = current_pos[0] + distance * np.cos(np.radians(angle))
        y = current_pos[1] + distance * np.sin(np.radians(angle))

        ax.scatter(x, y, c='black', s=0.5)  # Plot the new point


# Update rover's position based on its action
def update_position(current_pos, current_action, speed, time_step):
    """Update rover's position based on current action."""
    if current_action == 'forward':
        current_pos[0] += speed * np.cos(np.radians(current_pos[2])) * time_step
        current_pos[1] += speed * np.sin(np.radians(current_pos[2])) * time_step
    elif current_action == 'backward':
        current_pos[0] -= speed * np.cos(np.radians(current_pos[2])) * time_step
        current_pos[1] -= speed * np.sin(np.radians(current_pos[2])) * time_step
    elif current_action == 'left':
        current_pos[2] += 90  # Turning left (increasing angle)
    elif current_action == 'right':
        current_pos[2] -= 90  # Turning right (decreasing angle)
    return current_pos


# Main function for autonomous driving
def autonomous_driving():
    """Main function to handle the autonomous driving logic."""
    lidar = RPLidar(PORT_NAME, baudrate=115200)
    rover = RoverControl(ROVER_IP)

    # Plot setup
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')

    current_pos = [0, 0, 0]  # Start at (0, 0) with angle 0 degrees (facing forward)
    speed = 0.1  # Speed in meters per second
    time_step = 0.1  # Update rate in seconds
    current_action = None

    try:
        print("Starting autonomous driving...")
        rover.stop()
        time.sleep(1)

        lidar.stop()
        lidar.stop_motor()
        time.sleep(1)
        lidar.start_motor()
        time.sleep(1)

        for i, scan in enumerate(lidar.iter_scans()):
            try:
                print(f"Scan {i}: {len(scan)} points")
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

                # Update map with new scan data
                update_map(scan, current_pos, ax)
                plt.pause(0.05)  # Pause to update plot

                # Update the rover's position
                current_pos = update_position(current_pos, current_action, speed, time_step)

            except Exception as e:
                print(f"Error during scan processing: {e}")
                if "Incorrect descriptor starting bytes" in str(e):
                    print("Lidar connection error detected, attempting to restart.")
                    restart_lidar(lidar)
                    continue  # Continue mapping after restarting

    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Stopping LIDAR and Rover...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        rover.stop()


# Run the autonomous driving logic
if __name__ == "__main__":
    autonomous_driving()

