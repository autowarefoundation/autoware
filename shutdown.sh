#!/bin/sh
echo shutdown script

echo ""disable gnss cmd " > /dev/ttyUSB0"
cat ./ros/src/sensing/drivers/gnss/packages/navsat/scripts/disable.txt > /dev/ttyUSB0 
