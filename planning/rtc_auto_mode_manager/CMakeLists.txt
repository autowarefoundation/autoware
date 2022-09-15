cmake_minimum_required(VERSION 3.5)
project(rtc_auto_mode_manager)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rtc_auto_mode_manager_interface.cpp
  src/node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rtc_auto_mode_manager::RTCAutoModeManagerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
