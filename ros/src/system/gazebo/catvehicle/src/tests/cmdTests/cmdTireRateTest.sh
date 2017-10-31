#!/bin/bash

echo "Tires right"
rostopic pub -1 /catvehicle/front_left_steering_controller/command std_msgs/Float64 "data: -1.0" &
rostopic pub -1 /catvehicle/front_right_steering_controller/command std_msgs/Float64 "data: -1.0" 

echo "Tires front"
rostopic pub -1 /catvehicle/front_left_steering_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub -1 /catvehicle/front_right_steering_controller/command std_msgs/Float64 "data: 0.0" 

echo "Tires left"
rostopic pub -1 /catvehicle/front_left_steering_controller/command std_msgs/Float64 "data: 1.0" &
rostopic pub -1 /catvehicle/front_right_steering_controller/command std_msgs/Float64 "data: 1.0" 

echo "Tires front"
rostopic pub -1 /catvehicle/front_left_steering_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub -1 /catvehicle/front_right_steering_controller/command std_msgs/Float64 "data: 0.0" 

