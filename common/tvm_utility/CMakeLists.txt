# Copyright 2021-2022 Arm Limited and Contributors.
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

cmake_minimum_required(VERSION 3.14)
project(tvm_utility)

include("${PROJECT_NAME}-extras.cmake")

find_package(autoware_cmake REQUIRED)
autoware_package()

# Library

set(TVM_UTILITY_NODE_LIB_HEADERS
  "include/${PROJECT_NAME}/pipeline.hpp"
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${TVM_UTILITY_NODE_LIB_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(BUILD_TESTING)
  # Get target backend
  set(${PROJECT_NAME}_BACKEND llvm CACHE STRING "${PROJECT_NAME} neural network backend")

  # compile each folder inside test/ as a test case
  find_package(ament_cmake_gtest REQUIRED)
  find_package(OpenCV REQUIRED)
  set(tvm_runtime_DIR ${tvm_vendor_DIR})
  find_package(tvm_runtime CONFIG REQUIRED)

  set(TEST_ARTIFACTS "${CMAKE_CURRENT_LIST_DIR}/artifacts")
  file(GLOB TEST_CASES test/*)
  foreach(TEST_FOLDER ${TEST_CASES})
    if(NOT IS_DIRECTORY ${TEST_FOLDER})
      continue()
    endif()
    # the folder name becomes the test case name
    file(RELATIVE_PATH TEST_CASE_NAME ${CMAKE_CURRENT_LIST_DIR}/test ${TEST_FOLDER})

    # Get neural network.
    set(NN_DEPENDENCY "")
    get_neural_network(${TEST_CASE_NAME} ${${PROJECT_NAME}_BACKEND} NN_DEPENDENCY)

    if(NOT NN_DEPENDENCY STREQUAL "")
      if(TEST_CASE_NAME STREQUAL "yolo_v2_tiny" AND
          NOT EXISTS ${TEST_ARTIFACTS}/yolo_v2_tiny/test_image_0.jpg)
        message(WARNING "Missing image artifact for yolo_v2_tiny, skipping test")
        continue()
      endif()
      # add all cpp files in the folder to the target
      file(GLOB TEST_CASE_SOURCES ${TEST_FOLDER}/*.cpp)
      ament_add_gtest(${TEST_CASE_NAME} ${TEST_CASE_SOURCES})
      ament_target_dependencies(${TEST_CASE_NAME}
        "ament_index_cpp"
        "tvm_vendor"
      )
      add_dependencies(${TEST_CASE_NAME} ${NN_DEPENDENCY})

      target_link_libraries("${TEST_CASE_NAME}"
        "${OpenCV_LIBRARIES}"
        "${tvm_runtime_LIBRARIES}"
      )

      target_include_directories("${TEST_CASE_NAME}" SYSTEM PUBLIC
        "${OpenCV_INCLUDE_DIRS}"
        "${tvm_utility_FOUND_INCLUDE_DIRS}"
        "data/models"
        "include"
      )

      # Install test-specific files
      if(IS_DIRECTORY ${TEST_ARTIFACTS}/${TEST_CASE_NAME})
        install(DIRECTORY ${TEST_ARTIFACTS}/${TEST_CASE_NAME}/
          DESTINATION ${CMAKE_BINARY_DIR}/${TEST_CASE_NAME}_artifacts
        )
      endif()

    else()
      message(WARNING "No model is generated for ${TEST_FOLDER}, skipping test")
    endif()

  endforeach()
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake")
ament_auto_package()
