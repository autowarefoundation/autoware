#!/bin/sh

if [ "$1" = "CarDemo" ]
then
  roslaunch car_demo demo.launch
elif [ "$1" = "CatVechicle" ]
then
  roslaunch catvehicle catvehicle_skidpan.launch &
  gzclient &
  roslaunch point_cloud_converter point_cloud_converter.launch &
  roslaunch laser_scan_converter laser_scan_converter.launch &
  roslaunch twist_cmd_converter twist_cmd_converter.launch &
elif [ "$1" = "TestMap1" ]
then
  #this is to be implemented
  gazebo
fi

while :; do sleep 10; done

# EOF
