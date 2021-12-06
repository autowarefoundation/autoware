cmake_minimum_required(VERSION 3.5)
project(autoware_debug_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

#ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "scripts/")

ament_auto_add_library(lateral_error_publisher SHARED
  src/lateral_error_publisher.cpp
)

rclcpp_components_register_node(lateral_error_publisher
  PLUGIN "LateralErrorPublisher"
  EXECUTABLE lateral_error_publisher_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)

install(PROGRAMS scripts/stop_reason2pose.py scripts/pose2tf.py scripts/tf2pose.py
                scripts/case_converter.py scripts/self_pose_listener.py
                scripts/stop_reason2tf DESTINATION lib/${PROJECT_NAME})

install(FILES DESTINATION share/${PROJECT_NAME})
