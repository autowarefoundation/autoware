# Copyright 2018 the Autoware Foundation
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

project(autoware_auto_cmake NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "autoware_auto_cmake-extras.cmake"
)

if(BUILD_TESTING)
    ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_auto_package(INSTALL_TO_SHARE
  cmake
)
