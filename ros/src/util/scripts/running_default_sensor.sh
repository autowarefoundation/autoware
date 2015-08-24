#!/bin/bash

echo "load ros user settings..."
source devel/setup.bash

echo "run sensor driver..."
gnome-terminal --tab --title "hdl64" -e "sh -c 'roslaunch velodyne velodyne_hdl64e.launch;exec bash'" --tab --title "gnss" -e "sh -c 'sleep 3s;rosrun javad gnss.sh;exec bash'" --tab --title "grasshopper" -e "sh -c 'sleep 6s;roslaunch pointgrey grasshopper3.launch;exec bash'"
