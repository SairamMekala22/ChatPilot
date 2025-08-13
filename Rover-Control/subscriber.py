import paho.mqtt.client as paho

def on_message(mosq, obj, msg):
    print(msg.topic + ", " + str(msg.qos) + ", " + str(msg.payload))
    if msg.topic == "chatpilot/rover/move":
        print("Moving rover by " + str(msg.payload))
    mosq.publish('pong', 'ack', 0)

def on_publish(mosq, obj, mid):
    pass

if __name__ == '__main__':
    client = paho.Client()
    client.on_message = on_message
    client.on_publish = on_publish

    # client.tls_set('root.ca', certfile='c1.crt', keyfile='c1.key')
    client.connect("13.232.191.178", 1883, 60)

    client.subscribe("chatpilot/rover", 0)
    client.subscribe("chatpilot/#", 0)

    while client.loop() == 0:
        pass