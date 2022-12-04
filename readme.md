Robot driver
============

Wifi heavy duty servo motor controller

https://user-images.githubusercontent.com/1014092/205401948-c3db500c-c33b-486b-9b15-0c6e07ba4ee2.mp4

![Rear view](rear.jpg)


Features
--------

- 3 bidirectional motors (6 output channels in total)
- 3 encoder input channels for motor position.
- Wireless low-latency interface via MQTT
- Battery powered with a 14.4V Makita power tool battery.


Tech
----

- ESP32 controller
- one BTS???? driver for each channel (6 total)
- Inrush current limiter via low-side series transistor

PCB fits the case of a Makita battery charger.


