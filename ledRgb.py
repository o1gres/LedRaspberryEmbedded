#!/usr/bin/env python
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time
 
#MANAGE COLOR


pins = {'pin_R':11, 'pin_G':15, 'pin_B':18}  # pins is a dict
 
GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
for i in pins:
        GPIO.setup(pins[i], GPIO.OUT)   # Set pins' mode is output
        GPIO.output(pins[i], GPIO.HIGH) # Set pins to high(+3.3V) to off led
	#GPIO.output(pins[i], 0) # Set pins to high(+3.3V) to off le
 
p_R = GPIO.PWM(pins['pin_R'], 2000)  # set Frequece to 2KHz
p_G = GPIO.PWM(pins['pin_G'], 2000)
p_B = GPIO.PWM(pins['pin_B'], 2000)
 
p_R.start(0)      # Initial duty Cycle = 0(leds off)
p_G.start(0)
p_B.start(0)
 
def map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
 
def setColor(col):   # For example : col = 0x112233
	col = "0x"+col
        print("ricevuto il colore: " + col + " accendo i led")


        print("red0: ",int(col, 16))
        print("green0: ",int(col, 16))
        print("blue0: ",int(col, 16))
	

        R_val = int(hex((int(col, 16) & 0x110000) >> 16), 16)
	G_val = int(hex((int(col, 16) & 0x001100) >> 8), 16)
	B_val = int(hex((int(col, 16) & 0x000011) >> 0), 16)



#	R = (0x25caca & 0x110000) >> 16
#	G = (0x25caca & 0x001100) >> 8
#        B = (0x25caca & 0x000011) >> 0

#	print("R: ",R)
#	print("G: ",G)
#	print("B: ",B)

 #       print("red1: ",R_val)
 #       print("green1: ",G_val)
 #       print("blue1: ",B_val)

 
      #  R_val = map(R_val, 0, 255, 0, 100)
      #  G_val = map(G_val, 0, 255, 0, 100)
      #  B_val = map(B_val, 0, 255, 0, 100)
	
	print("red2: ",R_val)
	print("green2: ",G_val)
	print("blue2: ",B_val)
 

        p_R.ChangeDutyCycle(100-R_val)     # Change duty cycle
        p_G.ChangeDutyCycle(100-G_val)
        p_B.ChangeDutyCycle(100-B_val)



def setLedColor(color):

        try:
                if (color == 'off'):

                        print("ricevuto off spengo i led")
                        for i in pins:
                                #GPIO.output(pins[i], 0)    # Turn off all leds
				GPIO.output(pins[i], GPIO.HIGH)

#                        GPIO.cleanup()

                else:
                        setColor(color)

        except KeyboardInterrupt:
                p_R.stop()
                p_G.stop()
                p_B.stop()

                for i in pins:
                        GPIO.output(pins[i], GPIO.HIGH)    # Turn off all leds
#			GPIO.output(pins[i], 0)    # Turn off all leds
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







