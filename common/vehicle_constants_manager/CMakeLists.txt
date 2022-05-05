cmake_minimum_required(VERSION 3.14)
project(vehicle_constants_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(VEHICLE_CONSTANTS_MANAGER_LIB_SRC
  src/vehicle_constants_manager.cpp)

set(VEHICLE_CONSTANTS_MANAGER_LIB_HEADERS
  include/vehicle_constants_manager/vehicle_constants_manager.hpp
  include/vehicle_constants_manager/visibility_control.hpp)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${VEHICLE_CONSTANTS_MANAGER_LIB_SRC}
  ${VEHICLE_CONSTANTS_MANAGER_LIB_HEADERS})

# if(BUILD_TESTING)
#   set(TEST_SOURCES test/test_vehicle_constants_manager.cpp)
#   set(TEST_VEHICLE_CONSTANTS_MANAGER_EXE test_vehicle_constants_manager)
#   ament_add_gtest(${TEST_VEHICLE_CONSTANTS_MANAGER_EXE} ${TEST_SOURCES})
#   target_link_libraries(${TEST_VEHICLE_CONSTANTS_MANAGER_EXE} ${PROJECT_NAME})
# endif()

ament_auto_package(INSTALL_TO_SHARE)
