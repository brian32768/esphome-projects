I followed the instructions on the [ESPHome Getting Started page](https://esphome.io/guides/getting_started_command_line.html)

The docker version works fine on Bellman (Linux) and on Mac until the upload stage. Since the ESP32 is plugged into the Mac,
I cannot do a serial upload from Bellman, and Docker + USB fails on the Mac. So I am using Conda instead.

```bash
 conda create --name=esphome python
 conda activate esphome
 pip install esphome
``` 

Create a YAML file

   esphome wizard wrover2.yaml
or
   docker run --rm -v "${PWD}":/config -it esphome/esphome wrover2.yaml wizard

Process the YAML file, will attempt OTA upgrade on the Docker version, because it can't find the serial port.

   esphome run wrover2.yaml
or
   docker run --rm -v "${PWD}":/config -it esphome/esphome wrover2.yaml run

Process the YAML and do upgrade over USB port on Linux; this step fails on the Mac because
it will run a virtual machine that cannot see the serial ports.

   docker run --rm -v "${PWD}":/config --device=/dev/tty.usbserial-1410 -it esphome/esphome wrover2.yaml run


192.168.123.201	   wrover1
192.168.123.202	   wrover2 
192.168.123.209      m5stick01
192.168.123.240      wemos_rfid

BLE devices

ESYB E4:FD     battery charger
F1:1D:85:35:B5:00 Charge 2 Brian
F1:FA:1E:DC:4B:08 Charge 2 Julie
E0:75:52:DC:71:D3 Charge 3 Julie

DD:19:D7:96:C3:10
F7:45:4C:C&:03:90

CC:EE:76:72:A0:1F Tile Alba key
Tile Brian house keys
Tile Julie wallet
Tile Julie keys