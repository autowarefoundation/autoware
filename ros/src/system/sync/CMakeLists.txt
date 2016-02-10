cmake_minimum_required(VERSION 2.8.3)
project(synchronization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_tracker
  lidar_tracker
  geometry_msgs
  visualization_msgs
  points2image
)
find_package(OpenCV REQUIRED)

#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
add_message_files(
  FILES
  time_monitor.msg
  time_diff.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS cv_tracker points2image
)
###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sync_drivers sync_drivers.cpp)
target_link_libraries(sync_drivers ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(sync_drivers synchronization_generate_messages_cpp)

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
add_dependencies(time_monitor synchronization_generate_messages_cpp)