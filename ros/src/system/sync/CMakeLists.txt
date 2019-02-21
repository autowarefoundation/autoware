cmake_minimum_required(VERSION 2.8.3)
project(synchronization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  lidar_tracker
  geometry_msgs
  visualization_msgs
  points2image
  autoware_msgs
)
find_package(OpenCV REQUIRED)

#set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS autoware_msgs points2image message_runtime std_msgs
)
###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

add_executable(sync_drivers sync_drivers.cpp)
target_link_libraries(sync_drivers ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(sync_drivers ${catkin_EXPORTED_TARGETS})

add_executable(sync_range_fusion computing/perception/detection/packages/cv_tracker/nodes/range_fusion/sync_range_fusion.cpp)
target_link_libraries(sync_range_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sync_track computing/perception/detection/packages/cv_tracker/nodes/kf_track/sync_track.cpp)
target_link_libraries(sync_track ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sync_obj_reproj computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/sync_obj_reproj.cpp)
target_link_libraries(sync_obj_reproj ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sync_obj_fusion computing/perception/detection/packages/lidar_tracker/nodes/obj_fusion/sync_obj_fusion.cpp)
target_link_libraries(sync_obj_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(time_monitor time_monitor.cpp)
target_link_libraries(time_monitor ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(time_monitor ${catkin_EXPORTED_TARGETS})

        
install(TARGETS
        sync_drivers
        sync_range_fusion
        sync_track
        sync_obj_reproj
        sync_obj_fusion
        time_monitor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
