cmake_minimum_required(VERSION 3.14)
project(component_interface_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_auto_add_executable(service_log_checker src/service_log_checker.cpp)
ament_auto_package(INSTALL_TO_SHARE launch)
