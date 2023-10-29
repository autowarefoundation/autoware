cmake_minimum_required(VERSION 3.14)
project(ar_tag_based_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

ament_auto_add_executable(ar_tag_based_localizer
  src/main.cpp
  src/ar_tag_based_localizer.cpp
)
target_include_directories(ar_tag_based_localizer
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ar_tag_based_localizer ${OpenCV_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_ar_tag_based_localizer
    test/test.cpp
    src/ar_tag_based_localizer.cpp
  )
  target_include_directories(test_ar_tag_based_localizer
    SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_ar_tag_based_localizer ${OpenCV_LIBRARIES})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
