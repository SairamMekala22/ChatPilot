import serial
import json
import time

# ----- Serial Port Settings -----

SERIAL_PORT = '/dev/ttyACM1'  # May be /dev/ttyUSB0 or /dev/ttyACM1 — check with 'ls /dev/ttyACM\*'
BAUDRATE = 115200

# Open serial connection to ESP32 on Driver HAT

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
def send_motor_command(motor_id, velocity_rpm):

    """
    Send velocity command to DDSM115 motor via ESP32.

    :param motor_id: int — motor ID (1 or 2)
    :param velocity_rpm: float — target velocity in RPM

    """

    command = {
        "cmd": "control",
        "id": motor_id,
        "mode": "speed",    # velocity control mode
        "value": velocity_rpm  # target speed in RPM
    }

    json_cmd = json.dumps(command) + '\\n'
    ser.write(json_cmd.encode('utf-8'))
    print(f"Sent to Motor {motor_id}: {json_cmd.strip()}")
    time.sleep(0.05)  # small delay to allow processing

try:
    while True:
        # Example: Set Motor 1 and Motor 2 to different speeds
        send_motor_command(1, 30)   # Motor 1 → 100 RPM
        send_motor_command(2, -100)  # Motor 2 → -100 RPM (reverse)
        time.sleep(2)  # wait 2 seconds

        # Stop motors
        send_motor_command(1, 0)     # Motor 1 → stop
        send_motor_command(2, 0)     # Motor 2 → stop
        time.sleep(2)  # wait before repeating

except KeyboardInterrupt:
    print("Stopping motors...")

    # On exit — stop both motors

    send_motor_command(1, 0)
    send_motor_command(2, 0)
    ser.close()
