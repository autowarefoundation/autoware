# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(trajectory_follower_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LATERAL_CONTROLLER_NODE lateral_controller_node)
ament_auto_add_library(${LATERAL_CONTROLLER_NODE} SHARED
  include/trajectory_follower_nodes/lateral_controller_node.hpp
  src/lateral_controller_node.cpp
)

autoware_set_compile_options(${LATERAL_CONTROLLER_NODE})
# TODO(lateral_controller) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(lateral_controller) : Temporary workaround until this is fixed.
target_compile_options(${LATERAL_CONTROLLER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${LATERAL_CONTROLLER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_nodes::LateralController"
  EXECUTABLE ${LATERAL_CONTROLLER_NODE}_exe
)

set(LONGITUDINAL_CONTROLLER_NODE longitudinal_controller_node)
ament_auto_add_library(${LONGITUDINAL_CONTROLLER_NODE} SHARED
  include/trajectory_follower_nodes/longitudinal_controller_node.hpp
  src/longitudinal_controller_node.cpp
)

autoware_set_compile_options(${LONGITUDINAL_CONTROLLER_NODE})
# TODO(longitudinal_controller) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(longitudinal_controller) : Temporary workaround until this is fixed.
target_compile_options(${LONGITUDINAL_CONTROLLER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${LONGITUDINAL_CONTROLLER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_nodes::LongitudinalController"
  EXECUTABLE ${LONGITUDINAL_CONTROLLER_NODE}_exe
)

set(LATLON_MUXER_NODE latlon_muxer_node)
ament_auto_add_library(${LATLON_MUXER_NODE} SHARED
  include/trajectory_follower_nodes/latlon_muxer_node.hpp
  src/latlon_muxer_node.cpp
)

autoware_set_compile_options(${LATLON_MUXER_NODE})
# TODO(latlon_muxer) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(latlon_muxer) : Temporary workaround until this is fixed.
target_compile_options(${LATLON_MUXER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${LATLON_MUXER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_nodes::LatLonMuxer"
  EXECUTABLE ${LATLON_MUXER_NODE}_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Unit tests
  set(TRAJECTORY_FOLLOWER_NODES_TEST test_trajectory_follower_nodes)
  ament_add_gtest(${TRAJECTORY_FOLLOWER_NODES_TEST}
    test/trajectory_follower_test_utils.hpp
    test/test_latlon_muxer_node.cpp
    test/test_lateral_controller_node.cpp
    test/test_longitudinal_controller_node.cpp
  )
  ament_target_dependencies(${TRAJECTORY_FOLLOWER_NODES_TEST} fake_test_node)
  autoware_set_compile_options(${TRAJECTORY_FOLLOWER_NODES_TEST})
  target_link_libraries(${TRAJECTORY_FOLLOWER_NODES_TEST} ${LATLON_MUXER_NODE} ${LATERAL_CONTROLLER_NODE} ${LONGITUDINAL_CONTROLLER_NODE})

  #find_package(autoware_testing REQUIRED)
  #add_smoke_test(${PROJECT_NAME} ${LATLON_MUXER_NODE}_exe PARAM_FILENAME latlon_muxer_defaults.yaml)
  #add_smoke_test(${PROJECT_NAME} ${LATERAL_CONTROLLER_NODE}_exe PARAM_FILENAME lateral_controller_defaults.yaml)
  #add_smoke_test(${PROJECT_NAME} ${LONGITUDINAL_CONTROLLER_NODE}_exe PARAM_FILENAME longitudinal_controller_defaults.yaml)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  param
)
