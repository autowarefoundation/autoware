#!/bin/bash

set -e

source ~/autoware/ros/devel/setup.bash
source ~/autoware/ros/src/apex_ros1/as_vehicle/install/setup.bash --extend
roslaunch demo_scripts all_sensors.launch
