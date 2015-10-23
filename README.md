APM-Mavlink-to-FrSky
=============================
Code for reading Mavlink data from the uart2 serial port of the APM2+ with a Arduino pro mini
and convert to FrSky telemetry protocol.

This a fork of the https://github.com/vizual54/APM-Mavlink-to-FrSky project with several changes:
* Compatibility with newer versions of Arduino IDE (tested with 1.6.3)
* Various fixes for several telemetry values, compatible and tested with [OpenTX](https://github.com/opentx/opentx) 2.0.x and 2.1.x series.

The firmware is used with Arduino Pro Mini 5V and APM mini 3.1 loaded with Ardupilot 3.2.1

See project homepage http://vizual54.github.io/APM-Mavlink-to-FrSky/ for information on how to connect the Arduino to the APM and to the FrSky RX.

