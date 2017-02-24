#!/bin/sh

roslaunch catvehicle catvehicle_skidpan.launch &
gzclient &
roslaunch point_cloud_converter point_cloud_converter.launch &
roslaunch laser_scan_converter laser_scan_converter.launch &
roslaunch twist_cmd_converter twist_cmd_converter.launch &

while :; do sleep 10; done

# EOF
