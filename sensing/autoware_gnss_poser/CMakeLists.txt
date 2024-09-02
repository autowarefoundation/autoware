cmake_minimum_required(VERSION 3.14)
project(autoware_gnss_poser)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

set(GNSS_POSER_HEADERS
  include/autoware/gnss_poser/gnss_poser_node.hpp
)

ament_auto_add_library(gnss_poser_node SHARED
  src/gnss_poser_node.cpp
  ${GNSS_POSER_HEADERS}
)

target_link_libraries(gnss_poser_node
  Geographic
)

rclcpp_components_register_node(gnss_poser_node
  PLUGIN "autoware::gnss_poser::GNSSPoser"
  EXECUTABLE gnss_poser
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
