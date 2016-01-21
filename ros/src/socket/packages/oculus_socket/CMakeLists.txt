cmake_minimum_required(VERSION 2.8.3)
project(oculus_socket)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(oculus_sender nodes/oculus_sender/oculus_sender.cpp)
target_link_libraries(oculus_sender ${catkin_LIBRARIES})
