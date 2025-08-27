import cv2
import paho.mqtt.client as mqtt
import time
import base64

# ========== MQTT SETTINGS ==========
BROKER = "10.11.102.28"   # Replace with your broker IP/hostname
PORT = 1883
TOPIC = "video/stream"

# ========== GStreamer PIPELINE ==========
# For CSI camera (Raspberry Pi Camera Module / Jetson Camera)
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )

# ========== MQTT CLIENT ==========
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Connected to MQTT Broker!")
    else:
        print("❌ Failed to connect, return code %d\n", rc)

client = mqtt.Client()
client.on_connect = on_connect

print("Connecting to broker...")
client.connect(BROKER, PORT, 60)
client.loop_start()

# ========== CAMERA CAPTURE ==========
# For CSI camera:
cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, framerate=90/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)


# If using USB camera, comment above and uncomment this:
# cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Error: Could not open camera.")
    exit()

print("📷 Starting video capture...")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to grab frame")
            break

        # Encode frame to JPEG
        _, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')

        # Publish frame
        client.publish(TOPIC, jpg_as_text)
        print("📤 Frame published")

        time.sleep(0.1)  # Limit frame rate (10 FPS)

except KeyboardInterrupt:
    print("🛑 Stopped by user")

finally:
    cap.release()
    client.loop_stop()
    client.disconnect()
    print("✅ Cleaned up and closed")
