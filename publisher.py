import paho.mqtt.publish as publish

msgs = [
    {'topic': "chatpilot/rover/move", 'payload': "forward 5"},
    {'topic': "chatpilot/rover/camera", 'payload': "capture"}
]

host = "13.232.191.178"
port = 1883

if __name__ == '__main__':
    # publish a single message
    publish.single(topic="chatpilot/rover/takeoff", payload="10", hostname=host, port=port)

    # publish multiple messages
    # publish.multiple(msgs, hostname=host)
