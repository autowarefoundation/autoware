cmake_minimum_required(VERSION 3.5)
project(freespace_planning_algorithms)

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

ament_auto_add_library(reeds_shepp SHARED
  src/reeds_shepp.cpp
)

ament_auto_add_library(freespace_planning_algorithms SHARED
  src/abstract_algorithm.cpp
  src/astar_search.cpp
)

target_link_libraries(freespace_planning_algorithms
  reeds_shepp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(freespace_planning_algorithms-test
    test/src/test_freespace_planning_algorithms.cpp
  )
  target_link_libraries(freespace_planning_algorithms-test
    freespace_planning_algorithms
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)

install(PROGRAMS
  test/debug_plot.py
  DESTINATION lib/${PROJECT_NAME}
)
