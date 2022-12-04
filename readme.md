Robot driver
============

Wifi heavy duty servo motor controller

**⚠️ Status: prototype ⚠️**

https://user-images.githubusercontent.com/1014092/205401948-c3db500c-c33b-486b-9b15-0c6e07ba4ee2.mp4

![rear](https://user-images.githubusercontent.com/1014092/205522420-85adf3dc-d7ae-46b3-922f-ef1716646d62.jpeg)


Features
--------

- 3 bidirectional motor outputs (motors wired as H-bridge; 6 output channels in total)
- 3 encoder input channels for motor position
- Wireless low-latency interface via MQTT
- Battery powered with a 14.4 V Makita power tool battery


Tech
----

- ESP32 controller
- BTS7960 driver for each output channel
- Inrush current limiter via low-side series MOSFET

PCB fits the case of a Makita battery charger.


Benchmarking
------------

Supply rail transient response during the demo program (as in the video):

![transients](https://user-images.githubusercontent.com/1014092/205522974-26e38b6e-1e94-4848-b9f2-b9c2e3c7a3d9.jpg)

![transients2](https://user-images.githubusercontent.com/1014092/205522979-7068d6bf-be71-4a43-afa0-1296b7fc97fe.jpg)


Credits
-------

This project was co-produced by V2_ Lab for the Unstable Media as part of the Winter Sessions art and technology residencies. ♥️
