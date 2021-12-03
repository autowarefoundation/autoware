cmake_minimum_required(VERSION 3.5)
project(gnss_poser)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_package(ament_cmake_auto)
ament_auto_find_build_dependencies()

set(GNSS_POSER_HEADERS
  include/gnss_poser/convert.hpp
  include/gnss_poser/gnss_poser_core.hpp
  include/gnss_poser/gnss_stat.hpp
)

ament_auto_add_library(gnss_poser_node SHARED
  src/gnss_poser_core.cpp
  ${GNSS_POSER_HEADERS}
)

target_link_libraries(gnss_poser_node
  Geographic
)

rclcpp_components_register_node(gnss_poser_node
  PLUGIN "gnss_poser::GNSSPoser"
  EXECUTABLE gnss_poser
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
