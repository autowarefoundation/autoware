cmake_minimum_required(VERSION 3.14)
project(diagnostic_graph_aggregator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/common/graph/error.cpp
  src/common/graph/data.cpp
  src/common/graph/config.cpp
  src/common/graph/loader.cpp
  src/common/graph/graph.cpp
  src/common/graph/units.cpp
)

ament_auto_add_executable(tree
  src/tool/tree.cpp
)
target_include_directories(tree PRIVATE src/common)

ament_auto_add_executable(plantuml
  src/tool/plantuml.cpp
)
target_include_directories(plantuml PRIVATE src/common)

ament_auto_add_library(aggregator SHARED
  src/node/aggregator.cpp
  src/node/availability.cpp
)
target_include_directories(aggregator PRIVATE src/common)

rclcpp_components_register_node(aggregator
  PLUGIN "diagnostic_graph_aggregator::AggregatorNode"
  EXECUTABLE aggregator_node
)

if(BUILD_TESTING)
  get_filename_component(RESOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/files ABSOLUTE)
  ament_auto_add_gtest(gtest_${PROJECT_NAME}
    test/src/test1.cpp
    test/src/test2.cpp
    test/src/utils.cpp
  )
  target_compile_definitions(gtest_${PROJECT_NAME} PRIVATE TEST_RESOURCE_PATH="${RESOURCE_PATH}")
  target_include_directories(gtest_${PROJECT_NAME} PRIVATE src/common)
endif()

ament_auto_package(INSTALL_TO_SHARE config example launch)
