cmake_minimum_required(VERSION 3.5)
project(dummy_perception_publisher)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies for messages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InitialState.msg"
  "msg/Object.msg"
  DEPENDENCIES autoware_auto_perception_msgs tier4_perception_msgs geometry_msgs std_msgs unique_identifier_msgs
)

# See ndt_omp package for documentation on why PCL is special
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(${PROJECT_NAME}_DEPENDENCIES
  autoware_auto_perception_msgs
  tier4_perception_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_auto_add_executable(dummy_perception_publisher_node
  src/main.cpp
  src/node.cpp
)

ament_target_dependencies(dummy_perception_publisher_node ${${PROJECT_NAME}_DEPENDENCIES})

target_include_directories(dummy_perception_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# For using message definitions from the same package
rosidl_target_interfaces(dummy_perception_publisher_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# PCL dependencies – `ament_target_dependencies` doesn't respect the
# components/modules selected above and only links in `common` ,so we need
# to do this manually.
target_compile_definitions(dummy_perception_publisher_node PRIVATE ${PCL_DEFINITIONS})
target_include_directories(dummy_perception_publisher_node PRIVATE ${PCL_INCLUDE_DIRS})
# Unfortunately, this one can't be PRIVATE because only the plain or only the
# keyword (PRIVATE) signature of target_link_libraries can be used for one
# target, not both. The plain signature is already used inside
# `ament_target_dependencies` and possibly rosidl_target_interfaces.
target_link_libraries(dummy_perception_publisher_node ${PCL_LIBRARIES})
target_link_directories(dummy_perception_publisher_node PRIVATE ${PCL_LIBRARY_DIRS})


ament_auto_add_executable(empty_objects_publisher
  src/empty_objects_publisher.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
