# Copyright 2021 The Autoware Foundation
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

cmake_minimum_required(VERSION 3.5)

project(osqp_interface)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
get_target_property(OSQP_INCLUDE_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(OSQP_INTERFACE_LIB_SRC
  src/csc_matrix_conv.cpp
  src/osqp_interface.cpp
)

set(OSQP_INTERFACE_LIB_HEADERS
  include/osqp_interface/csc_matrix_conv.hpp
  include/osqp_interface/osqp_interface.hpp
  include/osqp_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OSQP_INTERFACE_LIB_SRC}
  ${OSQP_INTERFACE_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast -Wno-error=useless-cast)

target_include_directories(osqp_interface PUBLIC "${OSQP_INCLUDE_DIR}")
ament_target_dependencies(osqp_interface
  Eigen3
  osqp_vendor
)

# crucial so downstream package builds because osqp_interface exposes osqp.hpp
ament_export_include_directories("${OSQP_INCLUDE_DIR}")
# crucial so the linking order is correct in a downstream package: libosqp_interface.a should come before libosqp.a
ament_export_libraries(osqp::osqp)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES
    test/test_osqp_interface.cpp
    test/test_csc_matrix_conv.cpp
  )
  set(TEST_OSQP_INTERFACE_EXE test_osqp_interface)
  ament_add_gtest(${TEST_OSQP_INTERFACE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_OSQP_INTERFACE_EXE})
  target_link_libraries(${TEST_OSQP_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
)
