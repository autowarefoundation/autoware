#!/bin/bash

set -e

source ~/autoware/autoware/ros/devel/setup.bash
source ~/autoware/autoware/ros/src/apex_ros1/as_vehicle/install/setup.bash --extend
source ~/autoware/ros1_bridge/install_isolated/ros1_bridge/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
