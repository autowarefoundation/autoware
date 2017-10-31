#!/bin/bash

echo "Move forward"
rostopic pub -1 /catvehicle/joint1_velocity_controller/command std_msgs/Float64 "data: 10.0" &
rostopic pub -1 /catvehicle/joint2_velocity_controller/command std_msgs/Float64 "data: 10.0" 

echo "Now stopping"
rostopic pub -1 /catvehicle/joint1_velocity_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub -1 /catvehicle/joint2_velocity_controller/command std_msgs/Float64 "data: 0.0" 

echo "Move backward"
rostopic pub -1 /catvehicle/joint1_velocity_controller/command std_msgs/Float64 "data: -10.0" &
rostopic pub -1 /catvehicle/joint2_velocity_controller/command std_msgs/Float64 "data: -10.0" 

echo "Now stopping"
rostopic pub -1 /catvehicle/joint1_velocity_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub -1 /catvehicle/joint2_velocity_controller/command std_msgs/Float64 "data: 0.0" 

