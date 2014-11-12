#!/bin/sh
rosparam set hokuyo_node/calibrate_time false
rosparam set hokuyo_node/port /dev/ttyACM0
rosrun hokuyo_node hokuyo_node
