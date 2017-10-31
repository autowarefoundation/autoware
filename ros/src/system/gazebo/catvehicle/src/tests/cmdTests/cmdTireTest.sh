#!/bin/bash

NS=catvehicle$1
CL=front_left_steering_position_controller
CR=front_right_steering_position_controller
R=10.0

echo "Tires right"
rostopic pub -1 /${NS}/${CL}/command std_msgs/Float64 "data: -${R}" &
rostopic pub -1 /${NS}/${CR}/command std_msgs/Float64 "data: -${R}" 

echo "Tires front"
rostopic pub -1 /${NS}/${CL}/command std_msgs/Float64 "data: 0.0" &
rostopic pub -1 /${NS}/${CR}/command std_msgs/Float64 "data: 0.0" 

echo "Tires left"
rostopic pub -1 /${NS}/${CL}/command std_msgs/Float64 "data: ${R}" &
rostopic pub -1 /${NS}/${CR}/command std_msgs/Float64 "data: ${R}" 

echo "Tires front"
rostopic pub -1 /${NS}/${CL}/command std_msgs/Float64 "data: 0.0" &
rostopic pub -1 /${NS}/${CR}/command std_msgs/Float64 "data: 0.0" 

