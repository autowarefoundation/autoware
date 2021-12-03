cmake_minimum_required(VERSION 3.5)
project(interpolation)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(interpolation SHARED
  src/linear_interpolation.cpp
  src/spline_interpolation.cpp
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_gtest(test_interpolation ${test_files})

  target_link_libraries(test_interpolation
    interpolation
  )
endif()

ament_auto_package()
