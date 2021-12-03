# Copyright 2019 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

cmake_minimum_required(VERSION 3.5)
project(autoware_auto_perception_rviz_plugin)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
ament_auto_find_build_dependencies()

set(OD_PLUGIN_LIB_SRC
  src/object_detection/detected_objects_display.cpp
  src/object_detection/tracked_objects_display.cpp
  src/object_detection/predicted_objects_display.cpp
)

set(OD_PLUGIN_LIB_HEADERS
  include/visibility_control.hpp
)
set(OD_PLUGIN_LIB_HEADERS_TO_WRAP
  include/object_detection/detected_objects_display.hpp
  include/object_detection/tracked_objects_display.hpp
  include/object_detection/predicted_objects_display.hpp
)

set(COMMON_HEADERS
  include/common/color_alpha_property.hpp
  include/object_detection/object_polygon_detail.hpp
  include/object_detection/object_polygon_display_base.hpp
)

set(COMMON_SRC
  src/common/color_alpha_property.cpp
  src/object_detection/object_polygon_detail.cpp
)

foreach(header "${OD_PLUGIN_LIB_HEADERS_TO_WRAP}")
  qt5_wrap_cpp(OD_PLUGIN_LIB_HEADERS_MOC "${header}")
endforeach()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${COMMON_HEADERS}
  ${COMMON_SRC}
  ${OD_PLUGIN_LIB_HEADERS}
  ${OD_PLUGIN_LIB_HEADERS_MOC}
  ${OD_PLUGIN_LIB_SRC}
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
)
target_include_directories(${PROJECT_NAME} PRIVATE include)

# Settings to improve the build as suggested on https://github.com/ros2/rviz/blob/ros2/docs/plugin_development.md
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "OBJECT_DETECTION_PLUGINS_BUILDING_LIBRARY")

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  icons
)
