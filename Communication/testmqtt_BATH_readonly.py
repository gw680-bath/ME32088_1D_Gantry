import paho.mqtt.client as mqtt
import time

# The callback for when the client receives a CONNACK response from the server.
# def on_connect(client, userdata, rc):
#     print("Connected with result code "+str(rc))
#     # Subscribing in on_connect() means that if we lose the connection and
#     # reconnect then subscriptions will be renewed.
#     client.subscribe("$SYS/#")

# This is a different function call based on this: http://www.steves-internet-guide.com/mqtt-python-callbacks/
def on_connect(client,userdata,flags,rc):
    print ("Connected with result code "+str(rc))
    #client.subscribe("$SYS/#")
    client.subscribe("TeamTopic")
    print("Subscribed to TeamTopic")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(str(time.time())+" "+msg.topic+" "+str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set("student",password="HousekeepingGlintsStreetwise")
client.connect("fesv-mqtt.bath.ac.uk",31415,60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface
client.loop_forever()
