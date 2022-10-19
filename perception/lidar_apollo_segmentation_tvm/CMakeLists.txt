# Copyright 2021-2022 Arm Ltd.
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
project(lidar_apollo_segmentation_tvm)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(tvm_runtime_DIR ${tvm_vendor_DIR})
find_package(tvm_runtime CONFIG REQUIRED)
find_package(PCL 1.10 REQUIRED)

# Used in "extras" template file
set(${PROJECT_NAME}_BUILT FALSE)

# Gather neural network information.
set(${PROJECT_NAME}_BACKEND llvm CACHE STRING "${PROJECT_NAME} neural network backend")
set(MODEL_NAME baidu_cnn)

# Get neural network.
set(NN_DEPENDENCY "")
get_neural_network(${MODEL_NAME} ${${PROJECT_NAME}_BACKEND} NN_DEPENDENCY)
if(NOT NN_DEPENDENCY STREQUAL "")
  # Library
  ament_auto_add_library(${PROJECT_NAME} SHARED
    data/models/${MODEL_NAME}/inference_engine_tvm_config.hpp
    include/lidar_apollo_segmentation_tvm/lidar_apollo_segmentation_tvm.hpp
    include/lidar_apollo_segmentation_tvm/cluster2d.hpp
    include/lidar_apollo_segmentation_tvm/disjoint_set.hpp
    include/lidar_apollo_segmentation_tvm/feature_generator.hpp
    include/lidar_apollo_segmentation_tvm/feature_map.hpp
    include/lidar_apollo_segmentation_tvm/log_table.hpp
    include/lidar_apollo_segmentation_tvm/util.hpp
    src/lidar_apollo_segmentation_tvm.cpp
    src/cluster2d.cpp
    src/feature_generator.cpp
    src/feature_map.cpp
    src/log_table.cpp
  )
  add_dependencies(${PROJECT_NAME} ${NN_DEPENDENCY})

  target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-sign-conversion" "-Wno-conversion")

  # Set includes as "SYSTEM" to avoid compiler errors on their headers.
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${PCL_INCLUDE_DIRS}"
    "${tvm_vendor_INCLUDE_DIRS}"
  )

  target_include_directories(${PROJECT_NAME} PRIVATE
    data/models
  )

  target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tvm_runtime_LIBRARIES}
  )

  if(BUILD_TESTING)
    # gtest
    set(LIDAR_APOLLO_SEGMENTATION_TVM_GTEST lidar_apollo_segmentation_tvm_gtest)
    ament_add_gtest(${LIDAR_APOLLO_SEGMENTATION_TVM_GTEST} test/main.cpp TIMEOUT 120)
    target_include_directories(${LIDAR_APOLLO_SEGMENTATION_TVM_GTEST} PRIVATE "include")
    target_link_libraries(${LIDAR_APOLLO_SEGMENTATION_TVM_GTEST} ${PROJECT_NAME})
  endif()

  ament_export_include_directories(${PCL_INCLUDE_DIRS})
  set(${PROJECT_NAME}_BUILT TRUE)
else()
  message(WARNING "Neural network not found, skipping package.")
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake.in")
ament_auto_package()
