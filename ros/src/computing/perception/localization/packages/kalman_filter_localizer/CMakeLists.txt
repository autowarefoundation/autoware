cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter_localizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  autoware_build_flags
  roscpp
  std_msgs
  tf
  tf2
  geometry_msgs
  autoware_msgs
  sensor_msgs
  rostest
  rosunit
)

find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES kalman_filter kalman_filter_delayed_measurement
 CATKIN_DEPENDS
   autoware_build_flags
   roscpp
   std_msgs
   tf
   tf2
   geometry_msgs
   autoware_msgs
   sensor_msgs
)

###########
## Build ##
###########

include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(kalman_filter lib/kalman_filter.cpp)
add_dependencies(kalman_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter ${catkin_LIBRARIES})

add_library(kalman_filter_delayed_measurement lib/kalman_filter_delayed_measurement.cpp)
add_dependencies(kalman_filter_delayed_measurement ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter_delayed_measurement ${catkin_LIBRARIES} kalman_filter)


add_executable(kalman_filter_localizer src/kalman_filter_localizer.cpp)
add_dependencies(kalman_filter_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter_localizer ${catkin_LIBRARIES} kalman_filter_delayed_measurement)


#############
## Install ##
#############

## Install executables and/or libraries
install(TARGETS kalman_filter kalman_filter_delayed_measurement kalman_filter_localizer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/kalman_filter/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        PATTERN ".svn" EXCLUDE)



#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test-kalman_filter
      test/test_kalman_filter.test
      test/src/test_kalman_filter.cpp
      lib/kalman_filter.cpp
      lib/kalman_filter_delayed_measurement.cpp
    )
    add_dependencies(test-kalman_filter ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-kalman_filter ${catkin_LIBRARIES})
endif ()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mpc_follower.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
