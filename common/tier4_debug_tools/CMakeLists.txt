cmake_minimum_required(VERSION 3.5)
project(tier4_debug_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(lateral_error_publisher SHARED
  src/lateral_error_publisher.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(lateral_error_publisher PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

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
