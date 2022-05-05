cmake_minimum_required(VERSION 3.14)
project(ndt)

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

add_library(ndt
  src/base.cpp
  src/pcl_generic.cpp
  src/pcl_modified.cpp
  src/omp.cpp
)

target_include_directories(ndt
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(ndt
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(ndt PUBLIC ndt_omp ndt_pcl_modified)
# Can't use ament_target_dependencies() here because it doesn't link PCL
# properly, see ndt_omp for more information.
target_compile_definitions(ndt PUBLIC ${PCL_DEFINITIONS})
target_include_directories(ndt PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(ndt PUBLIC ${PCL_LIBRARIES})
target_link_directories(ndt PUBLIC ${PCL_LIBRARY_DIRS})

ament_export_targets(export_ndt HAS_LIBRARY_TARGET)
ament_export_dependencies(ndt_omp ndt_pcl_modified PCL)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ndt
  EXPORT export_ndt
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
