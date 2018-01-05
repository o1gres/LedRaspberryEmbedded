#!/usr/bin/env python
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time


# The Pins. Use Broadcom numbers.
RED_PIN   = 17
GREEN_PIN = 22
BLUE_PIN  = 24

#MANAGE COLOR
def setColor(colore):
	red = int(colore[0:2], 16)
	green = int(colore[2:4], 16)
	blue = int(colore[4:6], 16)

	print("red: ", red)
        print("green: ", green)
        print("blue: ", blue)

	pi.set_PWM_dutycycle(RED_PIN, red)
	pi.set_PWM_dutycycle(GREEN_PIN, green)
	pi.set_PWM_dutycycle(BLUE_PIN, blue)



def setLedColor(color):

        try:
                if (color == 'off'):

                        print("ricevuto off spengo i led")
			setColor("000000")

                else:
                        setColor(color)

        except KeyboardInterrupt:
                p_R.stop()
                p_G.stop()
                p_B.stop()

                for i in pins:
                        GPIO.output(pins[i], GPIO.HIGH)    # Turn off all leds
                GPIO.cleanup()






#MQTT Connection
def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))


def on_message(mqttc, obj, msg):
    print('Received message: ' + str(msg.payload) + ' on topic: '+msg.topic )

    setLedColor(str(msg.payload))


def on_publish(mqttc, obj, mid):
    print("mid: " + str(mid))


def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " QoS: " + str(granted_qos))


def on_log(mqttc, obj, level, string):
    print(string)



mqttc = mqtt.Client("Raspberry Sergio", True, None, mqtt.MQTTv31)
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
# mqttc.on_log = on_log
mqttc.username_pw_set("sergio","password")
#mqttc.connect("127.0.0.1", 1883, 60)
mqttc.connect("64.137.238.149", 1883, 180)
mqttc.subscribe("SergioRaspberryLed", 2)

#mqttc.loop_forever()
mqttc.loop_forever(2,10)







