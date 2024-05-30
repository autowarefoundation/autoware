cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_planner_node)

find_package(autoware_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/LoadPlugin.srv"
  "srv/UnloadPlugin.srv"
  DEPENDENCIES
)

autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::motion_velocity_planner::MotionVelocityPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(${PROJECT_NAME}_lib
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")
endif()

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/src/test_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}_lib
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
