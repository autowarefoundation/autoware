cmake_minimum_required(VERSION 3.14)
project(tier4_metrics_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets Charts)
set(QT_WIDGETS_LIB Qt5::Widgets)
set(QT_CHARTS_LIB Qt5::Charts)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/metrics_visualize_panel.cpp
  include/metrics_visualize_panel.hpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_WIDGETS_LIB}
  ${QT_CHARTS_LIB}
)

target_compile_options(${PROJECT_NAME} PUBLIC -Wno-error=deprecated-copy -Wno-error=pedantic)
# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
