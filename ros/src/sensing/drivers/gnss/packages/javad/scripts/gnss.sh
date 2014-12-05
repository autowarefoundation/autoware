#!/bin/bash

echo ""em,,msg/nmea/GGA:0.04" > /dev/ttyUSB0"
echo "em,,msg/nmea/GGA:0.04" > /dev/ttyUSB0

echo "sleep 3 second"
sleep 3s

echo ""em,,msg/nmea/RMC:0.04" > /dev/ttyUSB0"
echo "em,,msg/nmea/RMC" > /dev/ttyUSB0

echo "sleep 3 second"
sleep 3s

echo "launch gnss driver..."
roslaunch javad nmea_navsat.launch
