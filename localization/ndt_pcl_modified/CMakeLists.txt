cmake_minimum_required(VERSION 3.5)
project(ndt_pcl_modified)

add_compile_options(-march=native)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(ndt_pcl_modified
  src/ndt.cpp
)

target_include_directories(ndt_pcl_modified
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ndt_pcl_modified PCL)
ament_export_targets(export_ndt_pcl_modified HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ndt_pcl_modified
  EXPORT export_ndt_pcl_modified
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
