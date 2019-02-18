cmake_minimum_required(VERSION 2.8.3)
project(garmin)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") #or


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  nmea_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velodyne_gps_imu
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(garmin_gps_18x_lvc src/garmin_gps_18x_lvc.cc)

target_link_libraries(garmin_gps_18x_lvc ${catkin_LIBRARIES})

install(TARGETS garmin_gps_18x_lvc
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})