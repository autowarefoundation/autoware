cmake_minimum_required(VERSION 3.14)
project(imu_corrector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(imu_corrector_node SHARED
  src/imu_corrector_core.cpp
  include/imu_corrector/imu_corrector_core.hpp
)

rclcpp_components_register_node(imu_corrector_node
  PLUGIN "imu_corrector::ImuCorrector"
  EXECUTABLE imu_corrector
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
