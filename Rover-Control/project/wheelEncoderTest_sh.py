from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import math

# === ROBOT PHYSICAL PARAMETERS ===
WHEEL_RADIUS = 0.05035  # in meters (50.35 mm)
ENCODER_RESOLUTION = 4096  # pulses per revolution
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_RESOLUTION
TARGET_DISTANCE = 1.0  # meters
TARGET_TICKS = int(TARGET_DISTANCE / DISTANCE_PER_TICK)

# === MODBUS CONFIGURATION ===
PORT = '/dev/ttyACM1' 
BAUDRATE = 115200

# === MOTOR ADDRESSES ===
FRONT_LEFT = 0x01
FRONT_RIGHT = 0x02
REAR_LEFT = 0x03
REAR_RIGHT = 0x04

ALL_MOTORS = [FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT]

# === MODBUS REGISTERS ===
ENCODER_REG = 0x01F4
SPEED_REG = 0x0200

# === SPEED SETTING ===
DRIVE_SPEED = 1000  # Adjust as needed

# === Initialize Modbus Client ===
client = ModbusClient(method='rtu', port=PORT, baudrate=BAUDRATE, timeout=1)
if not client.connect():
    raise Exception("Could not connect to DDSM Driver HAT via serial.")

def read_encoder(motor_id):
    result = client.read_holding_registers(ENCODER_REG, 2, unit=motor_id)
    if result.isError():
        print(f" Error reading encoder from motor {motor_id}")
        return 0
    value = (result.registers[0] << 16) | result.registers[1]
    return value - (1 << 32) if value >= 0x80000000 else value

def write_speed(motor_id, speed):
    if speed < 0:
        speed = (1 << 32) + speed
    high = (speed >> 16) & 0xFFFF
    low = speed & 0xFFFF
    client.write_registers(SPEED_REG, [high, low], unit=motor_id)

def stop_all():
    for motor in ALL_MOTORS:
        write_speed(motor, 0)

# === MAIN CONTROL LOGIC ===
try:
    print("Moving forward 1 meter (4WD)...")

    # Read starting encoder values (only front motors)
    left_start = read_encoder(FRONT_LEFT)
    right_start = read_encoder(FRONT_RIGHT)

    # Set all wheels to same speed
    for motor in ALL_MOTORS:
        write_speed(motor, DRIVE_SPEED)

    while True:
        left_now = read_encoder(FRONT_LEFT)
        right_now = read_encoder(FRONT_RIGHT)

        left_delta = abs(left_now - left_start)
        right_delta = abs(right_now - right_start)

        print(f"[Encoders] FL: {left_delta}, FR: {right_delta}")

        if left_delta >= TARGET_TICKS or right_delta >= TARGET_TICKS:
            break
        time.sleep(0.01)

finally:
    stop_all()
    client.close()
    print("Movement complete. All motors stopped.")

