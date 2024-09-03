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

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_update_module.cpp
  src/ndt_scan_matcher_core.cpp
  src/particle.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(ndt_scan_matcher ${PCL_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ndt_scan_matcher::NDTScanMatcher"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  add_launch_test(
    test/test_ndt_scan_matcher_launch.py
    TIMEOUT "30"
  )

  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(standard_sequence_for_initial_pose_estimation
    test/test_cases/standard_sequence_for_initial_pose_estimation.cpp
  )
  ament_auto_add_gtest(once_initialize_at_out_of_map_then_initialize_correctly
    test/test_cases/once_initialize_at_out_of_map_then_initialize_correctly.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
