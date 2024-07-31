cmake_minimum_required(VERSION 3.22)
project(autoware_control_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_control_validator_helpers SHARED
  src/utils.cpp
  src/debug_marker.cpp
)

# control validator
ament_auto_add_library(autoware_control_validator_component SHARED
  include/autoware/control_validator/control_validator.hpp
  src/control_validator.cpp
)
target_link_libraries(autoware_control_validator_component autoware_control_validator_helpers)
rclcpp_components_register_node(autoware_control_validator_component
  PLUGIN "autoware::control_validator::ControlValidator"
  EXECUTABLE autoware_control_validator_node
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ControlValidatorStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# to use a message defined in the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(autoware_control_validator_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(autoware_control_validator_component "${cpp_typesupport_target}")
endif()

if(BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_gtest(test_autoware_control_validator
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_control_validator autoware_control_validator_component)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
