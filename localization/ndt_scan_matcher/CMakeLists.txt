cmake_minimum_required(VERSION 3.5)
project(ndt_scan_matcher)

# Compile flags for SIMD instructions
# Be careful to change these options, especially when `ndt_omp` implementation is used.
# All packages linked to `ndt_omp` should use the same SIMD instruction set.
# In case mismatched instruction set are used, program causes a crash at its initialization
# because of a misaligned access to the `Eigen` libraries' data structure.
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  # For x86_64 architecture, SIMD instruction set is fixed below versions,
  # because the `ndt_omp` is optimized to these versions.
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
else()
  # For other architecture, like arm64, compile flags are generally prepared by compiler
  # march=native is disabled as default for specific depending pcl libraries
  # or pre-building packages for other computers.
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(ndt_scan_matcher
  src/debug.cpp
  src/ndt_scan_matcher_node.cpp
  src/ndt_scan_matcher_core.cpp
  src/util_func.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
