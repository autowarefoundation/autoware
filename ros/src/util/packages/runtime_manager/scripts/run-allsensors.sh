#!/bin/bash

set -e

source ~/autoware/autoware/ros/devel/setup.bash
source ~/autoware/autoware/ros/src/apex_ros1/as_vehicle/install/setup.bash --extend

#execute lidar tracker
bash ~/autoware/autoware/ros/src/apex_ros1/demo_scripts/scripts/run-lidartracker.sh &
sleep 10

# execute bridge
bash ~/autoware/autoware/ros/src/apex_ros1/demo_scripts/scripts/run-ros1bridge.sh &
sleep 5

#roslaunch demo_scripts all_sensors.launch
bash ~/autoware/autoware/ros/src/apex_ros1/demo_scripts/scripts/all_process.sh
