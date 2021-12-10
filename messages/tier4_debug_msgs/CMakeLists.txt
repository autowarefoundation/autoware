cmake_minimum_required(VERSION 3.5)
project(tier4_debug_msgs)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiArrayDimension.msg"
  "msg/MultiArrayLayout.msg"
  "msg/BoolStamped.msg"
  "msg/Float32Stamped.msg"
  "msg/Float32MultiArrayStamped.msg"
  "msg/Float64Stamped.msg"
  "msg/Float64MultiArrayStamped.msg"
  "msg/Int32Stamped.msg"
  "msg/Int32MultiArrayStamped.msg"
  "msg/Int64Stamped.msg"
  "msg/Int64MultiArrayStamped.msg"
  "msg/StringStamped.msg"
  DEPENDENCIES
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
