cmake_minimum_required(VERSION 3.14)
project(landmark_tf_caster)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

ament_auto_add_executable(landmark_tf_caster
  src/landmark_tf_caster_node.cpp
  src/landmark_tf_caster_core.cpp
)
target_include_directories(landmark_tf_caster
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
