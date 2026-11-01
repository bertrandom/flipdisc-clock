# flipdisc-clock

This is an alternative firmware for the [Flip-Disc Clock](https://github.com/marcinsaj/Flipo-Clock-4x7-Segment-Flip-Disc-Display). The original clock had an Arduino Nano Every, I've swapped that out with Arduino Nano ESP32 which allows it to connect to the Internet and my local LAN.

Added features:
* Web configuration
* Ability to update via OTA
* Clock syncs with NTP at startup and every hour
* Subscribes to MQTT
  * Listens to `iot/flipdisc` for words/numbers and displays them immediately on the display, after 10 seconds it will revert to the time
  * Publishes the temperature/humidity to `iot/flipdisc/state` once a minute
  * Publishes online status to `iot/flipdisc/status`
* Shows git version

This is a rewrite of [flipdisc-clock-arduino](https://github.com/bertrandom/flipdisc-clock-arduino) which was just an Arduino sketch, this is a PlatformIO project. Many of the above features didn't exist in the previous version.

## Configuration

Copy `include/arduino_secrets.example.h` to `include/arduino_secrets.h`.

Fill in your WiFi and MQTT details.

Copy `secrets.example.ini` to `secrets.ini`.

Generate a new OTA password:
```
openssl rand -hex 16
```

and put that in `secrets.ini`.

## Installation

The first flash has to happen over USB:

```
pio run -t upload
```

Subsequent flashes can happen over OTA:

```
pio run -e ota
```

OTA host is defined in `platformio.ini`, it must be changed if the IP address/hostname is incorrect.