cmake_minimum_required(VERSION 3.14)
project(system_diagnostic_graph)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/core/config.cpp
  src/core/debug.cpp
  src/core/graph.cpp
  src/core/units.cpp
  src/core/modes.cpp
)

ament_auto_add_executable(aggregator
  src/main.cpp
)

ament_auto_add_executable(converter
  src/tool.cpp
)

ament_auto_add_executable(tool
  src/tool/main.cpp
)
target_include_directories(tool PRIVATE src/core)

if(BUILD_TESTING)
  get_filename_component(RESOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/files ABSOLUTE)
  ament_auto_add_gtest(gtest_${PROJECT_NAME}
    test/src/test1.cpp
    test/src/test2.cpp
    test/src/utils.cpp
  )
  target_compile_definitions(gtest_${PROJECT_NAME} PRIVATE TEST_RESOURCE_PATH="${RESOURCE_PATH}")
  target_include_directories(gtest_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE config example launch)
