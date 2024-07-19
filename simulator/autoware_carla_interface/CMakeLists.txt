cmake_minimum_required(VERSION 3.8)
project(autoware_carla_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(
  catkin REQUIRED COMPONENTS std_msgs)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_export_dependencies(rclpy)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})


# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


ament_auto_package(
  launch
  resource
  config
  src
)
ament_package()
