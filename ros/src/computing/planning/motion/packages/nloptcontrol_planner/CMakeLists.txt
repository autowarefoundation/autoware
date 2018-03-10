cmake_minimum_required(VERSION 2.8.3)
project(nloptcontrol_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
  message_generation
  geometry_msgs
)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Control.msg)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Turn into a Python module using setup.py
catkin_python_setup()

## Generate added messages and services with any dependencies
generate_messages(DEPENDENCIES std_msgs)

catkin_package()
