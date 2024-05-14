cmake_minimum_required(VERSION 3.14)
project(smart_mpc_trajectory_follower)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(PROGRAMS
  smart_mpc_trajectory_follower/scripts/pympc_trajectory_follower.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package(
  INSTALL_TO_SHARE
)
