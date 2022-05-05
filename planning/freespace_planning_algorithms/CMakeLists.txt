cmake_minimum_required(VERSION 3.14)
project(freespace_planning_algorithms)

find_package(autoware_cmake REQUIRED)
autoware_package()

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
