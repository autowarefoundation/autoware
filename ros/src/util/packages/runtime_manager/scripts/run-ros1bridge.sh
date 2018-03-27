#!/bin/bash

set -e

source ~/autoware/ros/devel/setup.bash
source ~/autoware/ros/src/apex_ros1/as_vehicle/install/setup.bash --extend
source ~/ros1_bridge/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
