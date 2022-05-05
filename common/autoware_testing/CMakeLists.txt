cmake_minimum_required(VERSION 3.14)
project(autoware_testing)

find_package(autoware_cmake REQUIRED)
autoware_package()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "autoware_testing-extras.cmake"
)

ament_auto_package(
  INSTALL_TO_SHARE cmake autoware_testing
)
