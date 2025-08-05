import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar

# === Configuration ===
PORT = '/dev/ttyUSB0'  # Change if needed (e.g. '/dev/ttyUSB1')
MAX_DISTANCE = 4000    # mm (4 meters max display range)
MIN_DISTANCE = 100     # mm (ignore close noise)

# === Initialize Lidar ===
lidar = RPLidar(PORT)
print("Lidar connected on", PORT)

# === Set up plot ===
plt.ion()  # interactive mode on
fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_title('RPLidar A1 Live Scan')
scat = ax.scatter([], [], s=5, c='red')  # scatter plot for points

try:
    print("Starting Lidar scan...")
    for scan in lidar.iter_scans():
        angles = []
        distances = []

        for (_, angle, distance) in scan:
            if MIN_DISTANCE < distance < MAX_DISTANCE:
                angle_rad = np.radians(angle)
                x = distance * np.cos(angle_rad)
                y = distance * np.sin(angle_rad)
                angles.append(x)
                distances.append(y)

        scat.set_offsets(np.c_[angles, distances])
        plt.pause(0.001)  # brief pause to update plot

except KeyboardInterrupt:
    print("Stopping...")

finally:
    print("Cleaning up...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    plt.ioff()
    plt.show()
