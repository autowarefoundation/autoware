cmake_minimum_required(VERSION 3.14)
project(tier4_dummy_object_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Declare a C++ library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tools/util.cpp
  src/tools/interactive_object.cpp
  src/tools/pedestrian_pose.cpp
  src/tools/car_pose.cpp
  src/tools/unknown_pose.cpp
  src/tools/delete_all_objects.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES})

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
