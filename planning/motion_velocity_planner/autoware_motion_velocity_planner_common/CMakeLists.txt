cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_planner_common)

find_package(autoware_cmake REQUIRED)

autoware_package()

# ament_auto_add_library(${PROJECT_NAME}_lib SHARED
#   DIRECTORY src
# )

ament_auto_package(INSTALL_TO_SHARE
  include
)
