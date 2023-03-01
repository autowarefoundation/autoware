cmake_minimum_required(VERSION 3.14)
project(fault_injection)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(fault_injection_node_component SHARED
  src/fault_injection_node/fault_injection_node.cpp
)

rclcpp_components_register_node(fault_injection_node_component
  PLUGIN "fault_injection::FaultInjectionNode"
  EXECUTABLE fault_injection_node
)

if(BUILD_TESTING)
  # gtest
  ament_add_ros_isolated_gtest(test_fault_injection_node_component
    test/src/main.cpp
    test/src/test_diagnostic_storage.cpp
  )

  target_link_libraries(test_fault_injection_node_component
    fault_injection_node_component
  )

  # launch_testing
  add_launch_test(
    test/test_fault_injection_node.test.py
    TARGET test_fault_injection_launch_test
  )
  # TODO(KeisukeShima): Remove in Humble
  set_tests_properties(
    test_fault_injection_launch_test
    PROPERTIES
    LABELS "launch_test"
  )

  install(DIRECTORY
    test/config
    test/launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
