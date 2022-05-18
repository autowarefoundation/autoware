cmake_minimum_required(VERSION 3.14)
project(default_ad_api)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME} "default_ad_api::InterfaceNode")

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)
endif()

install(
  PROGRAMS script/web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch test)
