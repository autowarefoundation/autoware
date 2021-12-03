cmake_minimum_required(VERSION 3.5)
project(gyro_odometer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(GYRO_ODOMETER_SRC
  src/gyro_odometer_core.cpp)

set(GYRO_ODOMETER_HEADERS
  include/gyro_odometer/gyro_odometer_core.hpp)

ament_auto_add_executable(${PROJECT_NAME}
  src/gyro_odometer_node.cpp
  ${GYRO_ODOMETER_SRC}
  ${GYRO_ODOMETER_HEADERS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
