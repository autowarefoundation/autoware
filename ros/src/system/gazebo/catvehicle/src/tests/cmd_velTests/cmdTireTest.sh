#!/bin/bash

NS=/$1

echo "Tires right"
rostopic pub -1 $NS/cmd_vel geometry_msgs/Twist '{linear : {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}'

echo "Tires front"
rostopic pub -1 $NS/cmd_vel geometry_msgs/Twist '{linear : {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

echo "Tires left"
rostopic pub -1 $NS/cmd_vel geometry_msgs/Twist '{linear : {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'

echo "Tires front"
rostopic pub -1 $NS/cmd_vel geometry_msgs/Twist '{linear : {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

