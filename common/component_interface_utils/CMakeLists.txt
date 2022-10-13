cmake_minimum_required(VERSION 3.14)
project(component_interface_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_export_dependencies(tier4_system_msgs)
ament_auto_package()
