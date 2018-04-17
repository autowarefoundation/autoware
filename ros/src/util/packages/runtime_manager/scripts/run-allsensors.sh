#!/bin/bash

set -e

source ~/autoware/autoware/ros/devel/setup.bash
source ~/autoware/autoware/ros/src/apex_ros1/as_vehicle/install/setup.bash --extend
#roslaunch demo_scripts all_sensors.launch
bash ~/autoware/autoware/ros/src/apex_ros1/demo_scripts/scripts/all_process.sh
