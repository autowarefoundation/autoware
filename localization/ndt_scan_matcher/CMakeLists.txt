cmake_minimum_required(VERSION 3.14)
project(ndt_scan_matcher)

find_package(autoware_cmake REQUIRED)
autoware_package()

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

find_package(PCL REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})

ament_auto_add_executable(ndt_scan_matcher
  src/debug.cpp
  src/ndt_scan_matcher_node.cpp
  src/ndt_scan_matcher_core.cpp
  src/util_func.cpp
  src/pose_array_interpolator.cpp
  src/tf2_listener_module.cpp
  src/map_module.cpp
  src/pose_initialization_module.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(ndt_scan_matcher ${PCL_LIBRARIES})

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
