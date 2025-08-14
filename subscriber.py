import paho.mqtt.client as paho
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

def on_message(mosq, obj, msg):
    print(msg.topic + ", " + str(msg.qos) + ", " + str(msg.payload))
    if msg.topic == "chatpilot/rover/takeoff":
        arm(vehicle=vehicle)
        takeoff(vehicle=vehicle, altitude=int(msg.payload))
    mosq.publish('pong', 'ack', 0)

def on_publish(mosq, obj, mid):
    pass

def arm(vehicle):
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
        print(" Getting ready to take off ...")
        time.sleep(1)

def takeoff(altitude, vehicle):
    vehicle.simple_takeoff(altitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=altitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
vehicle_port = "tcp:127.0.0.1:5762"


if __name__ == '__main__':
    global vehicle
    client = paho.Client()
    client.on_message = on_message
    client.on_publish = on_publish

    # client.tls_set('root.ca', certfile='c1.crt', keyfile='c1.key')
    client.connect("13.232.191.178", 1883, 60)
    vehicle = connect(vehicle_port, wait_ready = True)
    client.subscribe("chatpilot/rover", 0)
    client.subscribe("chatpilot/#", 0)

    while client.loop() == 0:
        pass