cmake_minimum_required(VERSION 3.14)
project(autoware_motion_utils)

option(BUILD_EXAMPLES "Build examples" OFF)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)
find_package(fmt REQUIRED)

ament_auto_add_library(autoware_motion_utils SHARED
  DIRECTORY src
)

target_link_libraries(autoware_motion_utils
  fmt::fmt
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_autoware_motion_utils ${test_files})

  target_link_libraries(test_autoware_motion_utils
    autoware_motion_utils
  )
endif()

if(BUILD_EXAMPLES)
  message(STATUS "Building examples")

  include(FetchContent)
  fetchcontent_declare(
    matplotlibcpp17
    GIT_REPOSITORY https://github.com/soblin/matplotlibcpp17.git
    GIT_TAG master
  )
  fetchcontent_makeavailable(matplotlibcpp17)

  file(GLOB_RECURSE example_files examples/*.cpp)

  foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    ament_auto_add_executable(${example_name} ${example_file})
    target_link_libraries(${example_name}
      autoware_motion_utils
      matplotlibcpp17::matplotlibcpp17
    )
  endforeach()
endif()

ament_auto_package()
