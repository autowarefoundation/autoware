cmake_minimum_required(VERSION 3.14)
project(map4_localization_launch)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(INSTALL_TO_SHARE
  eagleye_pose_twist_localization_launch
  eagleye_twist_localization_launch
)
