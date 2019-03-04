cmake_minimum_required(VERSION 2.8.3)
project(naive_motion_predict)

include(FindPkgConfig)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  imm_ukf_pda_track
  autoware_msgs
  )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        autoware_msgs
        tf
        imm_ukf_pda_track
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

#naive_motion_predict
add_executable(naive_motion_predict
        nodes/naive_motion_predict/naive_motion_predict_node.cpp
        nodes/naive_motion_predict/naive_motion_predict.cpp)

target_link_libraries(naive_motion_predict
        ${catkin_LIBRARIES}
        )

add_dependencies(naive_motion_predict
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS
        naive_motion_predict
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)