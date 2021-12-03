# Copyright 2020 Tier IV, Inc.
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

# - Find NVML
# Find the native NVML(NVIDIA Management Library) includes and libraries
#
#  NVML_INCLUDE_DIRS - where to find nvml.h.
#  NVML_LIBRARIES    - the library needed to use NVML.
#  NVML_FOUND        - True if NVML found.

if(NOT NVML_INCLUDE_DIRS)
  find_path(NVML_INCLUDE_DIRS nvml.h PATHS /usr/local/cuda/include)
endif()

if(NOT NVML_LIBRARIES)
  find_library(NVML_LIBRARIES NAMES nvidia-ml)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NVML DEFAULT_MSG NVML_LIBRARIES NVML_INCLUDE_DIRS)

message(STATUS "NVML include dir: ${NVML_INCLUDE_DIRS}")
message(STATUS "NVML library : ${NVML_LIBRARIES}")
