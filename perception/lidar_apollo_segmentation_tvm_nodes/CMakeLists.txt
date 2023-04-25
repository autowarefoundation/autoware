# Copyright 2021-2023 Arm Ltd.
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

cmake_minimum_required(VERSION 3.14)
project(lidar_apollo_segmentation_tvm_nodes)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Only build if the "core" package has been built.
if(lidar_apollo_segmentation_tvm_BUILT)
  # Set lidar_apollo_segmentation_tvm includes as "SYSTEM" to ignore compiler errors on PCL headers
  include_directories(SYSTEM "${lidar_apollo_segmentation_tvm_INCLUDE_DIRS}")

  # Library
  ament_auto_add_library(${PROJECT_NAME} SHARED
    include/lidar_apollo_segmentation_tvm_nodes/lidar_apollo_segmentation_tvm_node.hpp
    src/lidar_apollo_segmentation_tvm_node.cpp
  )
  target_link_libraries(${PROJECT_NAME} ${lidar_apollo_segmentation_tvm_LIBRARIES})

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::perception::lidar_apollo_segmentation_tvm_nodes::ApolloLidarSegmentationNode"
    EXECUTABLE ${PROJECT_NAME}_exe
  )

  if(BUILD_TESTING)
    add_ros_test(
      test/launch.test.py
      TIMEOUT "30"
    )
  endif()

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
  )
else()
  message(WARNING "lidar_apollo_segmentation_tvm not built, skipping package.")
  ament_auto_package()
endif()
