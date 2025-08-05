from dronekit import connect, VehicleMode
from pymavlink import mavutil
from pynput import keyboard
import time

# Connect to the Pixhawk (Adjust COM port or connection method as needed)
vehicle = connect('COM7', baud=57600, wait_ready=False)  # For Windows
# vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)  # For Linux

# RC channel mappings
THROTTLE_CH = 3  # Forward/Backward
STEERING_CH = 1  # Left/Right steering

# Steering values
CENTER = 1500
TURN_AMOUNT = 85  # Adjust for fine-tuned steering correction

# Throttle values
THROTTLE_NEUTRAL = 1500
THROTTLE_MAX = 2000
THROTTLE_MIN = 1000

current_throttle = THROTTLE_NEUTRAL
current_steering = CENTER
initial_yaw = None  # Stores the reference yaw

def send_rc_override(throttle, steering):
    """Sends RC override commands to Pixhawk"""
    vehicle.channels.overrides = {
        THROTTLE_CH: throttle,
        STEERING_CH: steering
    }

def get_yaw():
    """Reads current yaw from Pixhawk's IMU"""
    attitude = vehicle.attitude  # Roll, pitch, yaw in radians
    yaw_degrees = attitude.yaw * (180.0 / 3.14159)  # Convert radians to degrees
    return yaw_degrees

def correct_drift():
    """Corrects rover drift based on yaw deviation"""
    global current_steering
    yaw = get_yaw()

    if initial_yaw is None:
        return  # No correction needed if no reference yaw is set
    
    yaw_deviation = yaw - initial_yaw  # Check deviation from initial heading

    if yaw_deviation > 1:  # Drifting right, correct left
        current_steering = CENTER + TURN_AMOUNT
    elif yaw_deviation < -1:  # Drifting left, correct right
        current_steering = CENTER - TURN_AMOUNT
    else:
        current_steering = CENTER  # Stay centered if within threshold
    
    send_rc_override(current_throttle, current_steering)

def move_rover(direction, throttle, duration):
    """Moves the rover in the given direction with yaw correction"""
    global current_throttle, initial_yaw

    if direction == 'forward':
        current_throttle = THROTTLE_NEUTRAL + int(500 * (throttle / 100))  # Scale PWM (1500-2000)
    elif direction == 'backward':
        current_throttle = THROTTLE_NEUTRAL - int(500 * (throttle / 100))  # Scale PWM (1500-1000)
    else:
        print("Invalid direction. Use 'forward' or 'backward'.")
        return
    
    # Store initial yaw when movement starts
    initial_yaw = get_yaw()
    print(f"Initial Yaw: {initial_yaw:.2f}°")

    print(f"Moving {direction} with throttle {throttle}% for {duration} seconds")

    start_time = time.time()
    while time.time() - start_time < duration:
        correct_drift()  # Continuously adjust steering to maintain direction
        time.sleep(0.1)

    # Stop movement after duration
    current_throttle = THROTTLE_NEUTRAL
    send_rc_override(current_throttle, CENTER)
    time.sleep(1)
    vehicle.channels.overrides = {}  # Clear overrides
    print("Stopping rover movement.")

def main():
    """Main function"""
    print("Yaw-based auto-correction activated. Type 'exit' to quit.")

    while True:
        direction = input("Enter direction (forward/backward): ").strip().lower()
        if direction not in ["forward", "backward"]:
            print("Invalid direction. Try again.")
            continue
        
        try:
            throttle = int(input("Enter throttle percentage (0-100): "))
            duration = float(input("Enter movement duration in seconds: "))
        except ValueError:
            print("Invalid input. Please enter valid numbers.")
            continue
        
        move_rover(direction, throttle, duration)
        
        user_input = input("Type 'exit' to quit or press Enter to continue: ").strip().lower()
        if user_input == "exit":
            break
    
    print("Exiting...")
    vehicle.channels.overrides = {}  # Clear overrides
    vehicle.close()

if __name__ == "__main__":
    main()