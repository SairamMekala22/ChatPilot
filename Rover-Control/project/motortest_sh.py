import serial
import json
import time

# ----- Serial Port Settings -----
SERIAL_PORT = '/dev/ttyACM1'  # or check: /dev/ttyUSB0 depending on Jetson
BAUDRATE = 115200

# Open serial connection to ESP32 (on DDSM Driver HAT)
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

# ----- Function to Send Command -----
def send_motor_command(motor_id, velocity_rpm):

    """
    Sends velocity command to a specific DDSM115 motor via RS485 through ESP32.
    :param motor_id: int, motor address (1, 2, 3, or 4)
    :param velocity_rpm: float, target velocity in RPM
    """

    # Command JSON (based on Waveshare's JSON protocol)
    command = {
        "cmd": "control",
        "id": motor_id,
        "mode": "speed",       # mode: 'speed' for velocity control
        "value": velocity_rpm  # target speed in RPM
    }

    # Convert dict to JSON string and add newline (required by ESP32 parser)
    json_cmd = json.dumps(command) + '\\n'
    ser.write(json_cmd.encode('utf-8'))
    print(f"Sent to Motor {motor_id}: {json_cmd.strip()}")
    time.sleep(0.05)  # Short delay to allow processing

# ----- Example: Set Speeds -----
try:
    while True:
        # Set each motor to a different speed (RPM)
        send_motor_command(1, 50)   # Motor 1 → 50 RPM
        #send_motor_command(2, 100)  # Motor 2 → 100 RPM
        #send_motor_command(3, -50)  # Motor 3 → -50 RPM (reverse)
        #send_motor_command(4, 0)    # Motor 4 → 0 RPM (stop)

        time.sleep(1)  # wait before sending again (for demo)

except KeyboardInterrupt:

    print("Stopping all motors...")

    # Stop all motors before exit
    for motor_id in range(1, 5):
        send_motor_command(motor_id, 0)
    ser.close()
