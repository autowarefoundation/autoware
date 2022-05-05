cmake_minimum_required(VERSION 3.14)
project(gyro_odometer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/gyro_odometer_node.cpp
  src/gyro_odometer_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
