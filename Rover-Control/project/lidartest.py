from motorauto import RoverController  # Your existing class
from rplidar import RPLidar
import time

class AutonomousRover:
    def __init__(self, lidar_port, rover_ip):
        self.rover = RoverController(rover_ip)
        self.lidar = RPLidar(lidar_port)
        self.min_safe_distance = 100  # mm, tweak as needed

    def process_lidar_data(self):
        """
        Get LIDAR readings and make movement decisions.
        """
        for scan in self.lidar.iter_scans():
            front_distance = None

            for (_, angle, distance) in scan:
                # Filter points that are in front (example: angle 350 to 10 degrees)
                if angle <= 10 or angle >= 350:
                    if front_distance is None or distance < front_distance:
                        front_distance = distance

            print(f"Front Distance: {front_distance} mm")

            if front_distance is not None:
                if front_distance < self.min_safe_distance:
                    print("Obstacle detected! Turning right...")
                    self.rover.turn_right()
                    time.sleep(0.5)  # turn for a bit
                    self.rover.move_forward()
                else:
                    print("Path clear. Moving forward.")
                    self.rover.move_forward()
            else:
                print("No LIDAR data. Stopping.")
                self.rover.stop()

            time.sleep(0.1)

    def start(self):
        try:
            print("Starting autonomous mode...")
            self.process_lidar_data()
        except KeyboardInterrupt:
            print("Stopping autonomous mode...")
        finally:
            self.rover.stop()
            self.lidar.stop()
            self.lidar.disconnect()

# Example Usage                                                               
if __name__ == "__main__":
    rover_ip = "http://192.168.4.1/js"
    lidar_port = "/dev/ttyUSB0"  # Adjust depending on your Jetson Nano

    autonomous_rover = AutonomousRover(lidar_port, rover_ip)
    autonomous_rover.start()

