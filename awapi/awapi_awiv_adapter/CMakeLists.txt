cmake_minimum_required(VERSION 3.5)
project(awapi_awiv_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(awapi_awiv_adapter
  src/awapi_awiv_adapter_node.cpp
  src/awapi_awiv_adapter_core.cpp
  src/awapi_vehicle_state_publisher.cpp
  src/awapi_autoware_state_publisher.cpp
  src/awapi_stop_reason_aggregator.cpp
  src/awapi_v2x_aggregator.cpp
  src/awapi_lane_change_state_publisher.cpp
  src/awapi_obstacle_avoidance_state_publisher.cpp
  src/awapi_max_velocity_publisher.cpp
  src/awapi_autoware_util.cpp
  src/awapi_pacmod_util.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
