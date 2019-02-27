#!/bin/sh

roslaunch vehicle_gazebo_simulation_launcher gazebo_launcher.launch gpu:=true &

while :; do sleep 10; done

# EOF
