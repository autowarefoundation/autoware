cmake_minimum_required(VERSION 3.14)
project(object_recognition_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(object_recognition_utils SHARED
  src/predicted_path_utils.cpp
  src/conversion.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_object_recognition_utils ${test_files})

  target_link_libraries(test_object_recognition_utils
    object_recognition_utils
  )
endif()

ament_auto_package()
