#!/bin/bash

echo ""em,,/msg/nmea/GGA:0.1" > /dev/ttyUSB0"
echo "em,,/msg/nmea/GGA:0.1" > /dev/ttyUSB0

echo "sleep 1 second"
sleep 1s

echo ""em,,/msg/nmea/RMC:0.1" > /dev/ttyUSB0"
echo "em,,/msg/nmea/RMC:0.1" > /dev/ttyUSB0

echo "sleep 1 second"
sleep 1s


echo "launch gnss driver..."
roslaunch javad nmea_navsat.launch
