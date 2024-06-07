cmake_minimum_required(VERSION 3.14)
project(autoware_path_sampler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_path_sampler SHARED
  DIRECTORY src
)

# register node
rclcpp_components_register_node(autoware_path_sampler
  PLUGIN "autoware::path_sampler::PathSampler"
  EXECUTABLE path_sampler_exe
)

ament_auto_package(
#   INSTALL_TO_SHARE
)
