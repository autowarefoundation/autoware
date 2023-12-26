cmake_minimum_required(VERSION 3.14)
project(autoware_auto_geometry)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/autoware_auto_geometry/spatial_hash.hpp
  include/autoware_auto_geometry/intersection.hpp
  include/autoware_auto_geometry/spatial_hash_config.hpp
  src/spatial_hash.cpp
  src/bounding_box.cpp
)

if(BUILD_TESTING)
  set(GEOMETRY_GTEST geometry_gtest)
  set(GEOMETRY_SRC test/src/test_geometry.cpp
    test/src/test_convex_hull.cpp
    test/src/test_hull_pockets.cpp
    test/src/test_interval.cpp
    test/src/lookup_table.cpp
    test/src/test_area.cpp
    test/src/test_common_2d.cpp
    test/src/test_intersection.cpp
  )
  ament_add_ros_isolated_gtest(${GEOMETRY_GTEST} ${GEOMETRY_SRC})
  target_compile_options(${GEOMETRY_GTEST} PRIVATE -Wno-conversion -Wno-sign-conversion)
  target_include_directories(${GEOMETRY_GTEST} PRIVATE "test/include" "include")
  ament_target_dependencies(${GEOMETRY_GTEST}
    "autoware_auto_common"
    "autoware_auto_geometry_msgs"
    "autoware_auto_planning_msgs"
    "autoware_auto_vehicle_msgs"
    "geometry_msgs"
    "osrf_testing_tools_cpp")
  target_link_libraries(${GEOMETRY_GTEST} ${PROJECT_NAME})
endif()

ament_auto_package()
