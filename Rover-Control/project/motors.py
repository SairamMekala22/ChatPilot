import json
import serial
import time

DDSM_PORT = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200

ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)

# right 2 
# left 1 

while True:
    
    speed_left = 0
    speed_right = 0
    print(f"Speed left: {speed_left}, right: {speed_right}")

    command_left = {
        "T": 10010,
        "id": 1,
        "cmd": speed_left,  # reverse polarity for right wheel
        "act": 3
    }
    command_right = {
        "T": 10010,
        "id": 2,
        "cmd": -speed_right,
        "act": 3
    }

    ddsm_ser.write((json.dumps(command_right) + '\n').encode())
    time.sleep(0.01)
    ddsm_ser.write((json.dumps(command_left) + '\n').encode())
    time.sleep(0.1)
