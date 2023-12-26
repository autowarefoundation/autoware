cmake_minimum_required(VERSION 3.14)
project(tier4_planning_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(tier4_planning_rviz_plugin SHARED
  # path point
  include/tier4_planning_rviz_plugin/path/display_base.hpp
  include/tier4_planning_rviz_plugin/path/display.hpp
  src/path/display.cpp
  # footprint
  include/tier4_planning_rviz_plugin/pose_with_uuid_stamped/display.hpp
  src/pose_with_uuid_stamped/display.cpp
  include/tier4_planning_rviz_plugin/mission_checkpoint/mission_checkpoint.hpp
  src/mission_checkpoint/mission_checkpoint.cpp
  src/tools/jsk_overlay_utils.cpp
  src/tools/max_velocity.cpp
)

target_link_libraries(tier4_planning_rviz_plugin
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
