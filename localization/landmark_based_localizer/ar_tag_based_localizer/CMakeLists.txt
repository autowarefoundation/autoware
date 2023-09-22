cmake_minimum_required(VERSION 3.14)
project(ar_tag_based_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

ament_auto_add_executable(ar_tag_based_localizer
  src/ar_tag_based_localizer_node.cpp
  src/ar_tag_based_localizer_core.cpp
)
target_include_directories(ar_tag_based_localizer
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ar_tag_based_localizer ${OpenCV_LIBRARIES})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
