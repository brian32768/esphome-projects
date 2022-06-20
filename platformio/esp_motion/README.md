# esp_motion

I use this project to create a motion sensor that communicates over
WiFi with an MQTT broker.  My Home Assistant server subscribes to
messages from the ESP32 and can react when the doppler sensor detects
motion.

## Set up

Copy wifi-SAMPLE.h to wifi.h

Edit it to put in your WiFi credentials and your MQTT IP address,

Connect the motion sensor to the ESP32. Details here soon.

Upload it into a WeMOS R1 D1 or similar.

## What it does

On boot, connects to WiFI using credentials in wifi.h

It watches for input from the motion sensor and when it sees movement, broadcasts
a message via MQTT.

It accepts messages over MQTT,
An "OFF" message means stop blinking the LED when motion is sensed. 
"ON" means go back to blinking the LED when motion is sensed.
An RGBW message turns on the LED and sets the color and brightness.
A TONE message beeps the beeper at a given frequency and duration.

If you wanted to, you could use it as a night light by sending OFF
followed by RGBW. The LED will stay on. Of course there is also an
annoyingly bright LED that indicates the board is powered up.

## Hardware, currently

One of
* ESP32 Wrover
* WeMOS D1 R1 Arduino (based on the ESP8266)

* Doppler motion sensor ($1.95 from MPJA)
* PIR motion sensor (Adafruit, about $2)

On the ESP32 I've got an external antenna and a beeper and a Neopixel LED.
I like the Neopixel; I think that will become a standard feature.

## Infrastructure

I use Home Assistant and the Mosquitto MQTT broker.

## TODO

* Document how to configure in Home Assistant
* Remove unused MQTT PUB messages
* Document the hookup. It's so simple almost not worth it. Almost. :-)

