cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# pure_pursuit_core
ament_auto_add_library(pure_pursuit_core SHARED
  src/pure_pursuit_core/planning_utils.cpp
  src/pure_pursuit_core/pure_pursuit.cpp
  src/pure_pursuit_core/interpolate.cpp
)

# pure_pursuit
ament_auto_add_library(pure_pursuit_node SHARED
  src/pure_pursuit/pure_pursuit_node.cpp
  src/pure_pursuit/pure_pursuit_viz.cpp
)

target_link_libraries(pure_pursuit_node
  pure_pursuit_core
)

rclcpp_components_register_node(pure_pursuit_node
  PLUGIN "pure_pursuit::PurePursuitNode"
  EXECUTABLE pure_pursuit_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
