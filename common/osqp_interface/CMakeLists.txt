cmake_minimum_required(VERSION 3.14)
project(osqp_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

# after find_package(osqp_vendor) in ament_auto_find_build_dependencies
find_package(osqp REQUIRED)
get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)

set(OSQP_INTERFACE_LIB_SRC
  src/csc_matrix_conv.cpp
  src/osqp_interface.cpp
)

set(OSQP_INTERFACE_LIB_HEADERS
  include/osqp_interface/csc_matrix_conv.hpp
  include/osqp_interface/osqp_interface.hpp
  include/osqp_interface/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OSQP_INTERFACE_LIB_SRC}
  ${OSQP_INTERFACE_LIB_HEADERS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast -Wno-error=useless-cast)

target_include_directories(osqp_interface
  SYSTEM PUBLIC
    "${OSQP_INCLUDE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
)

ament_target_dependencies(osqp_interface
  Eigen3
  osqp_vendor
)

# crucial so downstream package builds because osqp_interface exposes osqp.hpp
ament_export_include_directories("${OSQP_INCLUDE_DIR}")
# crucial so the linking order is correct in a downstream package: libosqp_interface.a should come before libosqp.a
ament_export_libraries(osqp::osqp)

if(BUILD_TESTING)
  set(TEST_SOURCES
    test/test_osqp_interface.cpp
    test/test_csc_matrix_conv.cpp
  )
  set(TEST_OSQP_INTERFACE_EXE test_osqp_interface)
  ament_add_ros_isolated_gtest(${TEST_OSQP_INTERFACE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_OSQP_INTERFACE_EXE} ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
)
