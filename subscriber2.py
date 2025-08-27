#!/usr/bin/env python
# -- coding: utf-8 --
"""
Subscribe to MQTT topic and display live camera feed
"""
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import sys
import time
import base64  # Added this import

# MQTT broker details
BROKER = "10.11.102.28"  # Use localhost if broker is on this machine; otherwise use laptop's network IP
PORT = 1883
TOPIC = "video/stream"

# Global flag to control loop
running = True

# Callback when connected to broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(TOPIC)
    else:
        print(f"Connection failed with code {rc}")
        sys.exit(1)

# Callback for new messages
def on_message(client, userdata, msg):
    global running
    try:
        # Decode base64 to JPEG bytes
        img_b64 = msg.payload.decode("utf-8")
        img_data = base64.b64decode(img_b64)
        np_arr = np.frombuffer(img_data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow("Live Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                running = False
                client.disconnect()
        else:
            print("Error: Failed to decode frame")
    except Exception as e:
        print(f"Error processing message: {e}")

# Callback for disconnection
def on_disconnect(client, userdata, rc):
    global running
    if rc != 0:
        print(f"Unexpected disconnection with code {rc}")
    running = False

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

try:
    # Connect to broker
    print("Connecting to broker...")
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()  # Non-blocking loop

    print("Waiting for live video stream... Press 'q' to quit.")
    while running:
        time.sleep(0.01)  # Small delay to reduce CPU usage

except KeyboardInterrupt:
    print("Stopped by user")
finally:
    client.loop_stop()
    cv2.destroyAllWindows()
    print("Cleanup complete")