cmake_minimum_required(VERSION 3.5)
project(tier4_vehicle_rviz_plugin)

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

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

set(HEADERS
  src/tools/turn_signal.hpp
  src/tools/console_meter.hpp
  src/tools/steering_angle.hpp
  src/tools/jsk_overlay_utils.hpp
  src/tools/velocity_history.hpp
)

## Declare a C++ library
ament_auto_add_library(tier4_vehicle_rviz_plugin SHARED
  src/tools/turn_signal.cpp
  src/tools/console_meter.cpp
  src/tools/steering_angle.cpp
  src/tools/jsk_overlay_utils.cpp
  src/tools/velocity_history.cpp
  ${HEADERS}
)

target_link_libraries(tier4_vehicle_rviz_plugin
  ${QT_LIBRARIES})

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  images
  plugins
)
