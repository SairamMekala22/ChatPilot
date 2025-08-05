import serial
import json

DDSM_PORT = '/dev/ttyACM1'
SERIAL_BAUDRATE = 115200
ddsm_ser = serial.Serial(DDSM_PORT, baudrate=SERIAL_BAUDRATE)
ddsm_ser.setRTS(False)
ddsm_ser.setDTR(False)



def writeLeft(speed):
    global ddsm_ser
    command = {
        "T": 10010,
        "id": 2,
        "cmd": speed,
        "act": 3
    }
    ddsm_ser.write((json.dumps(command) + '\n').encode())

def writeRight(speed):
    global ddsm_ser
    command = {
        "T": 10010,
        "id": 1,
        "cmd": -speed,
        "act": 3
    }
    ddsm_ser.write((json.dumps(command) + '\n').encode())
