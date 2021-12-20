# Copyright 2021 the Autoware Foundation
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

# Set language standard with variables instead of per target so even targets that
# don't use autoware_set_compile_options() have this set when importing
# autoware_auto_cmake via ament_auto_find_build_dependencies()
if(NOT CMAKE_C_STANDARD)
  # Default to C11 (rcutils uses C11 thread local storage)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED ON)
  set(CMAKE_C_EXTENSIONS OFF)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Get user-provided variables
set(DOWNLOAD_ARTIFACTS OFF CACHE BOOL "enable artifact download")

# Clang tidy
if(TIDY_WITH_CLANG)
  string(CONCAT CMAKE_CXX_CLANG_TIDY
    "clang-tidy;"
    "-checks=-*,"
    "bugprone-*,"
    "cert-*,"
    "cppcoreguidelines-*,"
    "clang-analyze-*,"
    "google-*,"
    "hicpp-*,"
    "modernize-*,"
    "performance-*,"
    "readability-*")
endif()

# Try to adhere to strict ISO C++ as much as possible:
#    from https://lefticus.gitbooks.io/cpp-best-practices/content/02-Use_the_Tools_Available.html
function(autoware_set_compile_options target)

  if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    # Causes the visibility macros to use dllexport rather than dllimport,
    # which is appropriate when building the dll but not consuming it.
    string(TOUPPER ${target} PROJECT_NAME_UPPER)
    target_compile_options(${target} PRIVATE "/bigobj")
    target_compile_definitions(${target} PRIVATE
      ${PROJECT_NAME_UPPER}_BUILDING_DLL
      -D_CRT_NONSTDC_NO_WARNINGS
      -D_CRT_SECURE_NO_WARNINGS
      -D_WINSOCK_DEPRECATED_NO_WARNINGS)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${target} PRIVATE
      -Wall
      -Werror
      -Wextra
      #-Wshadow             # causes issues with ROS 2 headers
      #-Wnon-virtual-dtor   # causes issues with ROS 2 headers
      -pedantic
      -Wcast-align
      -Wunused
      -Wconversion
      -Wsign-conversion
      -Wdouble-promotion
      -Waddress
      #-Wnull-dereference    # gcc6
      #-Wduplicated-branches # gcc7
      #-Wduplicated-cond     # gcc6
      #-Wrestrict            # gcc7
      -fvisibility=hidden)
    # C++-only options
    target_compile_options(${target}
      PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Woverloaded-virtual -Wold-style-cast>)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${target}
      PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -Wuseless-cast>)
    target_compile_options(${target} PRIVATE -Wlogical-op -frecord-gcc-switches)
  endif()

endfunction()

# Turn off optimization compiler flags for `target` unless the variable `AUTOWARE_OPTIMIZATION_OF_SLOW_TARGETS` is
# defined and evaluates to `true` in a boolean context.
function(autoware_turn_off_optimization target)

  if(AUTOWARE_OPTIMIZATION_OF_SLOW_TARGETS)
    # do nothing
  else()
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(${target} PRIVATE -O0)
    endif()
  endif()

endfunction()
