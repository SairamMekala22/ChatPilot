import requests
import time

class RoverController:
    def __init__(self, base_url):
        self.base_url = base_url

    def send_command(self, motor_id, cmd, act=3):
        """
        Send command to a specific motor ID.
        """
        payload = {
            "T": 10010,
            "id": motor_id,
            "cmd": cmd,
            "act": act
        }

        params = {
            "json": str(payload).replace("'", '"')  # Convert dict to JSON string
        }

        try:
            response = requests.get(self.base_url, params=params)
            print(f"Motor {motor_id}: cmd={cmd}, Response: {response.text}")
        except requests.RequestException as e:
            print(f"Error sending command to motor {motor_id}: {e}")

    def move_forward(self, speed=10):
        print("Moving Forward")
        self.send_command(1, speed)
        self.send_command(2, -speed)

    def move_backward(self, speed=10):
        print("Moving Backward")
        self.send_command(1, -speed)
        self.send_command(2, speed)

    def turn_left(self, speed=10):
        print("Turning Left")
        self.send_command(1, -speed)
        self.send_command(2, -speed)

    def turn_right(self, speed=10):
        print("Turning Right")
        self.send_command(1, speed)
        self.send_command(2, speed)

    def stop(self):
        print("Stopping")
        self.send_command(1, 0)
        self.send_command(2, 0)


# Example Usage
if __name__ == "__main__":
    rover = RoverController("http://192.168.4.1/js")

    rover.move_backward(speed=100)
    time.sleep(3)
    rover.turn_right(speed=80)
    time.sleep(3)
    rover.move_backward(speed=100)
    time.sleep(2)
    rover.move_backward(speed=100)
    time.sleep(4)
    
    # rover.move_backward()
    # rover.turn_left()
    # rover.turn_right()
    rover.stop()

