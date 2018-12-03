#!/bin/bash

sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

roslaunch ymc g30esli_interface.launch

sudo ip link set can0 down
