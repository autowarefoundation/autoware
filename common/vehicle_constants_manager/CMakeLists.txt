cmake_minimum_required(VERSION 3.5)
project(vehicle_constants_manager)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(VEHICLE_CONSTANTS_MANAGER_LIB_SRC
  src/vehicle_constants_manager.cpp)

set(VEHICLE_CONSTANTS_MANAGER_LIB_HEADERS
  include/vehicle_constants_manager/vehicle_constants_manager.hpp
  include/vehicle_constants_manager/visibility_control.hpp)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${VEHICLE_CONSTANTS_MANAGER_LIB_SRC}
  ${VEHICLE_CONSTANTS_MANAGER_LIB_HEADERS})
autoware_set_compile_options(${PROJECT_NAME})

# Testing
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   # Unit tests
#   set(TEST_SOURCES test/test_vehicle_constants_manager.cpp)
#   set(TEST_VEHICLE_CONSTANTS_MANAGER_EXE test_vehicle_constants_manager)
#   ament_add_gtest(${TEST_VEHICLE_CONSTANTS_MANAGER_EXE} ${TEST_SOURCES})
#   autoware_set_compile_options(${TEST_VEHICLE_CONSTANTS_MANAGER_EXE})
#   target_link_libraries(${TEST_VEHICLE_CONSTANTS_MANAGER_EXE} ${PROJECT_NAME})
# endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE)
