#!/bin/bash

cwd=`dirname "${0}"`

echo ""enable command" > /dev/ttyUSB0"
cat $cwd/enable.txt > /dev/ttyUSB0

sleep 1

echo "launch gnss driver..."
roslaunch javad nmea_navsat.launch
