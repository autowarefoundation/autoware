cmake_minimum_required(VERSION 3.5)
project(ndt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(ndt_omp REQUIRED)
find_package(ndt_pcl_modified REQUIRED)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
