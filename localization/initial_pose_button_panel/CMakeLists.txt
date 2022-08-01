cmake_minimum_required(VERSION 3.14)
project(initial_pose_button_panel)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS -g)
set(CMAKE_AUTOMOC ON)

ament_auto_add_library(initial_pose_button_panel SHARED
  src/initial_pose_button_panel.cpp)
target_link_libraries(initial_pose_button_panel
  ${QT_LIBRARIES})

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
