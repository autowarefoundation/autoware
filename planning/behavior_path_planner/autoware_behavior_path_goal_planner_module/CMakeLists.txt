cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_goal_planner_module)

set(ament_cmake_lint_cmake_FOUND TRUE)

option(BUILD_EXAMPLES "Build examples" OFF)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pull_over_planner/freespace_pull_over.cpp
  src/pull_over_planner/geometric_pull_over.cpp
  src/pull_over_planner/shift_pull_over.cpp
  src/default_fixed_goal_planner.cpp
  src/goal_searcher.cpp
  src/util.cpp
  src/goal_planner_module.cpp
  src/manager.cpp
)

if(BUILD_EXAMPLES)
  message(STATUS "Building examples")
  include(FetchContent)
  FetchContent_Declare(
    matplotlibcpp17
    GIT_REPOSITORY https://github.com/soblin/matplotlibcpp17.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(matplotlibcpp17)

  file(GLOB_RECURSE example_files examples/*.cpp)

  foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    ament_auto_add_executable(${example_name} ${example_file})
    target_include_directories(${example_name} SYSTEM INTERFACE matplotlibcpp17::matplotlibcpp17)
    set_source_files_properties(${example_file} PROPERTIES COMPILE_FLAGS -Wno-error -Wno-attributes -Wno-unused-parameter)
    target_link_libraries(${example_name} matplotlibcpp17::matplotlibcpp17)
  endforeach()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    ${TEST_SOURCES}
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE config)
