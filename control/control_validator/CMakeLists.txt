cmake_minimum_required(VERSION 3.22)
project(control_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(control_validator_helpers SHARED
  src/utils.cpp
  src/debug_marker.cpp
)

# control validator
ament_auto_add_library(control_validator_component SHARED
  include/control_validator/control_validator.hpp
  src/control_validator.cpp
)
target_link_libraries(control_validator_component control_validator_helpers)
rclcpp_components_register_node(control_validator_component
  PLUGIN "control_validator::ControlValidator"
  EXECUTABLE control_validator_node
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ControlValidatorStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# to use a message defined in the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(control_validator_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(control_validator_component "${cpp_typesupport_target}")
endif()

# if(BUILD_TESTING)
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
