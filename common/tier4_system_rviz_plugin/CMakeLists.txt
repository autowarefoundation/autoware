cmake_minimum_required(VERSION 3.14)
project(tier4_system_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

set(HEADERS
  src/mrm_summary_overlay_display.hpp
  src/jsk_overlay_utils.hpp
)

## Declare a C++ library
ament_auto_add_library(tier4_system_rviz_plugin SHARED
  src/mrm_summary_overlay_display.cpp
  src/jsk_overlay_utils.cpp
  ${HEADERS}
)

target_link_libraries(tier4_system_rviz_plugin
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
