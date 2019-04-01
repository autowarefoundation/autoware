#!/bin/bash

sudo modprobe peak_usb

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 up

source ../../../../../devel/setup.bash

roslaunch sd sd_twizy_interface.launch
