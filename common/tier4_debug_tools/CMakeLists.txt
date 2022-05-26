cmake_minimum_required(VERSION 3.14)
project(tier4_debug_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(lateral_error_publisher SHARED
  src/lateral_error_publisher.cpp
)

rclcpp_components_register_node(lateral_error_publisher
  PLUGIN "LateralErrorPublisher"
  EXECUTABLE lateral_error_publisher_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)

install(PROGRAMS scripts/stop_reason2pose.py scripts/pose2tf.py scripts/tf2pose.py
                scripts/case_converter.py scripts/self_pose_listener.py
                scripts/stop_reason2tf DESTINATION lib/${PROJECT_NAME})

install(FILES DESTINATION share/${PROJECT_NAME})
