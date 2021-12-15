# Copyright 2020, The Autoware Foundation.
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

### Export headers
project(autoware_auto_tf2)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(test_tf2_autoware_auto_msgs test/test_tf2_autoware_auto_msgs.cpp)
  autoware_set_compile_options(test_tf2_autoware_auto_msgs)
  target_include_directories(test_tf2_autoware_auto_msgs PRIVATE "include")
  ament_target_dependencies(test_tf2_autoware_auto_msgs
    "autoware_auto_common"
    "autoware_auto_perception_msgs"
    "autoware_auto_system_msgs"
    "autoware_auto_geometry_msgs"
    "geometry_msgs"
    "orocos_kdl"
    "tf2"
    "tf2_ros"
)
endif()

ament_auto_package()
