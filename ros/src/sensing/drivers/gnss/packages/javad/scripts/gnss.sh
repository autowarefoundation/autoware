#!/bin/bash

echo ""em,,msg/nmea/GGA" > /dev/ttyUSB0"
echo "em,,msg/nmea/GGA" > /dev/ttyUSB0

echo "launch gnss driver..."
roslaunch javad nmea_navsat.launch
