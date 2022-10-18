cmake_minimum_required(VERSION 3.14)
project(freespace_planning_algorithms)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(reeds_shepp SHARED
  src/reeds_shepp.cpp
)

ament_auto_add_library(rrtstar_core SHARED
  src/rrtstar_core.cpp
)

target_link_libraries(rrtstar_core
  reeds_shepp
)

ament_auto_add_library(freespace_planning_algorithms SHARED
  src/abstract_algorithm.cpp
  src/astar_search.cpp
  src/rrtstar.cpp
)

target_link_libraries(freespace_planning_algorithms
  reeds_shepp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(freespace_planning_algorithms-test
    test/src/test_freespace_planning_algorithms.cpp
  )
  target_link_libraries(freespace_planning_algorithms-test
    freespace_planning_algorithms
  )

  find_package(ament_cmake_ros REQUIRED)
  ament_add_gtest(rrtstar_core_informed-test
    test/src/test_rrtstar_core_informed.cpp
  )
  target_link_libraries(rrtstar_core_informed-test
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
