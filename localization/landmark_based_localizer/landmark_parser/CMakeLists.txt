cmake_minimum_required(VERSION 3.14)
project(landmark_parser)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(landmark_parser
  src/landmark_parser_core.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
)
