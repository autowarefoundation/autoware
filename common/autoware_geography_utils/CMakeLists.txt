cmake_minimum_required(VERSION 3.14)
project(autoware_geography_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/height.cpp
  src/projection.cpp
  src/lanelet2_projector.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/*.cpp)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${test_files})

  target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  )
endif()

ament_auto_package()
