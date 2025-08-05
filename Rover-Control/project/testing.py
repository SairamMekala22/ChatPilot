import time
from rplidar import RPLidar

port = '/dev/ttyUSB0'
lidar = RPLidar(port)
info=lidar.get_info()
print(info)
lidar.stop()
