cmake_minimum_required(VERSION 2.8.3)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES runtime_manager
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

