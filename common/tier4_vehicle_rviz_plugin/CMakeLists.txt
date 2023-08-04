cmake_minimum_required(VERSION 3.14)
project(tier4_vehicle_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

set(HEADERS
  src/tools/turn_signal.hpp
  src/tools/console_meter.hpp
  src/tools/steering_angle.hpp
  src/tools/jsk_overlay_utils.hpp
  src/tools/velocity_history.hpp
  src/tools/acceleration_meter.hpp
)

## Declare a C++ library
ament_auto_add_library(tier4_vehicle_rviz_plugin SHARED
  src/tools/turn_signal.cpp
  src/tools/console_meter.cpp
  src/tools/steering_angle.cpp
  src/tools/jsk_overlay_utils.cpp
  src/tools/velocity_history.cpp
  src/tools/acceleration_meter.cpp
  ${HEADERS}
)

target_link_libraries(tier4_vehicle_rviz_plugin
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  images
  plugins
)
