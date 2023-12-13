cmake_minimum_required(VERSION 3.14)
project(autoware_auto_msgs_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(NODE_NAME ${PROJECT_NAME}_node)
set(EXEC_NAME ${PROJECT_NAME}_exe)
set(TEST_NAME test_${PROJECT_NAME})

ament_auto_add_library(${NODE_NAME}
  src/include/adapter_base.hpp
  src/include/adapter_base_interface.hpp
  src/include/adapter_control.hpp
  src/include/adapter_map.hpp
  src/include/adapter_perception.hpp
  src/include/adapter_planning.hpp
  src/include/autoware_auto_msgs_adapter_core.hpp
  src/autoware_auto_msgs_adapter_core.cpp)

rclcpp_components_register_node(${NODE_NAME}
  PLUGIN "autoware_auto_msgs_adapter::AutowareAutoMsgsAdapterNode"
  EXECUTABLE ${EXEC_NAME})

if(BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_ros_isolated_gtest(${TEST_NAME} ${TEST_SOURCES})
  target_include_directories(${TEST_NAME} PRIVATE src/include)
  target_link_libraries(${TEST_NAME} ${NODE_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config)
