cmake_minimum_required(VERSION 3.14)
project(scenario_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(scenario_selector_node SHARED
  src/scenario_selector_node/scenario_selector_node.cpp
)

rclcpp_components_register_node(scenario_selector_node
  PLUGIN "ScenarioSelectorNode"
  EXECUTABLE scenario_selector
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_node
  )
endif()


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
