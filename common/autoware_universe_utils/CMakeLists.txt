cmake_minimum_required(VERSION 3.14)
project(autoware_universe_utils)

option(BUILD_EXAMPLES "Build examples" OFF)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

find_package(fmt REQUIRED)

ament_auto_add_library(autoware_universe_utils SHARED
  src/geometry/alt_geometry.cpp
  src/geometry/geometry.cpp
  src/geometry/pose_deviation.cpp
  src/geometry/boost_polygon_utils.cpp
  src/geometry/random_convex_polygon.cpp
  src/geometry/random_concave_polygon.cpp
  src/geometry/gjk_2d.cpp
  src/geometry/sat_2d.cpp
  src/math/sin_table.cpp
  src/math/trigonometry.cpp
  src/ros/msg_operation.cpp
  src/ros/marker_helper.cpp
  src/ros/logger_level_configure.cpp
  src/system/backtrace.cpp
  src/system/time_keeper.cpp
  src/geometry/ear_clipping.cpp
)

target_link_libraries(autoware_universe_utils
  fmt::fmt
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_autoware_universe_utils ${test_files})

  target_link_libraries(test_autoware_universe_utils
    autoware_universe_utils
  )
endif()

if(BUILD_EXAMPLES)
  message(STATUS "Building examples")
  file(GLOB_RECURSE example_files examples/*.cpp)

  foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    add_executable(${example_name} ${example_file})
    target_link_libraries(${example_name}
      autoware_universe_utils
    )
    install(TARGETS ${example_name}
      DESTINATION lib/${PROJECT_NAME}
    )
  endforeach()
endif()

ament_auto_package()
