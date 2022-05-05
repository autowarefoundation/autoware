cmake_minimum_required(VERSION 3.5)
project(interpolation)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(interpolation SHARED
  src/linear_interpolation.cpp
  src/spline_interpolation.cpp
  src/spline_interpolation_points_2d.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(interpolation PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_gtest(test_interpolation ${test_files})

  target_link_libraries(test_interpolation
    interpolation
  )
endif()

ament_auto_package()
