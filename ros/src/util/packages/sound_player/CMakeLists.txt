cmake_minimum_required(VERSION 2.8.3)
project(sound_player)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
find_package(sound_play REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)
