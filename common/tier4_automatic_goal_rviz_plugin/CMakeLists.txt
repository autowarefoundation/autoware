cmake_minimum_required(VERSION 3.14)
project(tier4_automatic_goal_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/automatic_goal_sender.cpp
  src/automatic_goal_append_tool.cpp
  src/automatic_goal_panel.cpp
)

ament_auto_add_executable(automatic_goal_sender
  src/automatic_goal_sender.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  icons
  plugins
)
