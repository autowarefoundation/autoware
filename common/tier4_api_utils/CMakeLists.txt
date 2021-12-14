cmake_minimum_required(VERSION 3.5)
project(tier4_api_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  include_directories(include)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp tier4_external_api_msgs)
endif()

ament_auto_package()
