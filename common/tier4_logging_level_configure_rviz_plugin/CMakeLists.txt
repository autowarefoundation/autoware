cmake_minimum_required(VERSION 3.14)
project(tier4_logging_level_configure_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(tier4_logging_level_configure_rviz_plugin SHARED
  include/tier4_logging_level_configure_rviz_plugin/logging_level_configure.hpp
  src/logging_level_configure.cpp
)

target_link_libraries(tier4_logging_level_configure_rviz_plugin
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
  config
)
