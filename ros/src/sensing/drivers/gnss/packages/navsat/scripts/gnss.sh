#!/bin/bash

echo ""enable command" > /dev/ttyUSB0"
cat enable.txt > /dev/ttyUSB0

echo "launch gnss driver..."
roslaunch javad nmea_navsat.launch
