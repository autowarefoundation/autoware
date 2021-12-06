cmake_minimum_required(VERSION 3.5)
project(initial_pose_button_panel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS -g)
set(CMAKE_AUTOMOC ON)

ament_auto_add_library(initial_pose_button_panel SHARED
  src/initial_pose_button_panel.cpp)
target_link_libraries(initial_pose_button_panel
  ${QT_LIBRARIES})

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
