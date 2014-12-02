#!/bin/bash

echo "load ros user settings..."
source devel/setup.bash

echo "run velodyne HDL-64E S2 driver..."
gnome-terminal --geometry=20x20+0+0 --title "hdl64" -e "sh -c'roslaunch velodyne velodyne_hdl64e.launch'"

echo "run gnss driver..."
gnome-terminal --geometry=20x20+450+0 --title "gnss" -e "sh -c'rosrun javad gnss.sh'"

echo "run pointgrey driver..."
gnome-terminal --geometry=20x20+1000+0 --title "grasshopper" -e "sh -c'roslaunch pointgrey grasshopper3.launch'"
