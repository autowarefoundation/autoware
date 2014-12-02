#!/bin/bash

echo "load ros user settings..."
source devel/setup.bash

echo "run velodyne HDL-64E S2 driver..."
roslaunch velodyne velodyne_hdl64e.launch

echo "run gnss driver..."
rosrun javad gnss.sh

echo "run pointgrey driver..."
roslaunch pointgrey grasshopper3.launch
