cmake_minimum_required(VERSION 3.14)
project(tier4_debug_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(tier4_debug_rviz_plugin SHARED
  include/tier4_debug_rviz_plugin/float32_multi_array_stamped_pie_chart.hpp
  include/tier4_debug_rviz_plugin/jsk_overlay_utils.hpp
  src/float32_multi_array_stamped_pie_chart.cpp
  src/jsk_overlay_utils.cpp
)

target_link_libraries(tier4_debug_rviz_plugin
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
