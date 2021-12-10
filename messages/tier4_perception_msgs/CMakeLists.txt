cmake_minimum_required(VERSION 3.5)
project(tier4_perception_msgs)

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
  "msg/object_recognition/DetectedObjectWithFeature.msg"
  "msg/object_recognition/DetectedObjectsWithFeature.msg"
  "msg/object_recognition/DynamicObject.msg"
  "msg/object_recognition/DynamicObjectArray.msg"
  "msg/object_recognition/Feature.msg"
  "msg/object_recognition/PredictedPath.msg"
  "msg/object_recognition/Semantic.msg"
  "msg/object_recognition/Shape.msg"
  "msg/object_recognition/State.msg"
  DEPENDENCIES
    autoware_auto_perception_msgs
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
    unique_identifier_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
