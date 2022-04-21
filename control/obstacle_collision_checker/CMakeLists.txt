cmake_minimum_required(VERSION 3.5)
project(obstacle_collision_checker)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Target
## obstacle_collision_checker_node
ament_auto_add_library(obstacle_collision_checker SHARED
  src/obstacle_collision_checker_node/obstacle_collision_checker.cpp
  src/obstacle_collision_checker_node/obstacle_collision_checker_node.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(obstacle_collision_checker PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(obstacle_collision_checker
  PLUGIN "obstacle_collision_checker::ObstacleCollisionCheckerNode"
  EXECUTABLE obstacle_collision_checker_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
