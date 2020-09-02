from umqtt.simple import MQTTClient

SERVER = "mqtt.thingspeak.com"
client = MQTTClient("umqtt_client", SERVER)

CHANNEL_ID = "1131265"
WRITE_API_KEY = "S251B8I21QXCD7UB"

topic = "channels/" + CHANNEL_ID + "/publish/" + WRITE_API_KEY

payload = "field1=" + str(50)
client.connect()
client.publish(topic, payload)
client.disconnect() 