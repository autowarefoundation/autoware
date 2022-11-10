cmake_minimum_required(VERSION 3.14)
project(operation_mode_transition_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(operation_mode_transition_manager_node SHARED
  src/compatibility.cpp
  src/data.cpp
  src/node.cpp
  src/state.cpp
)

rclcpp_components_register_node(operation_mode_transition_manager_node
  PLUGIN "operation_mode_transition_manager::OperationModeTransitionManager"
  EXECUTABLE operation_mode_transition_manager_exe
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/OperationModeTransitionManagerDebug.msg"
  DEPENDENCIES builtin_interfaces
)

# to use same package defined message
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(operation_mode_transition_manager_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(operation_mode_transition_manager_node "${cpp_typesupport_target}")
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
