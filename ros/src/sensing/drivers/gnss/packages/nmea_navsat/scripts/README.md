# Launch file for nmea_navsat_driver

Serial port reader and parser for NMEA compatible GPS devices.

## Setup

```
$ sudo apt-get install ros-kinetic-nmea-navsat-driver
```

## ROS run

```
$ rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0 _baud:=57600
```
