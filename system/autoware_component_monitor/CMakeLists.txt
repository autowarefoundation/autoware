cmake_minimum_required(VERSION 3.8)
project(autoware_component_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/component_monitor_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::component_monitor::ComponentMonitor"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_unit_conversions test/test_unit_conversions.cpp)
  target_link_libraries(test_unit_conversions ${PROJECT_NAME})
  target_include_directories(test_unit_conversions PRIVATE src)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
