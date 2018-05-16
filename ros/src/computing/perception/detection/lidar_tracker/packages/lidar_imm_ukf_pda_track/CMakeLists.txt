cmake_minimum_required(VERSION 2.8.3)
project(lidar_imm_ukf_pda_track)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  autoware_msgs
  geometry_msgs
  tf
  jsk_recognition_msgs
  )


set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  autoware_msgs
  geometry_msgs
  tf
  jsk_recognition_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)

link_directories(${PCL_LIBRARY_DIRS})

#imm_ukf_pda
add_executable(imm_ukf_pda
  nodes/imm_ukf_pda/imm_ukf_pda_main.cpp
  nodes/imm_ukf_pda/imm_ukf_pda.cpp
  nodes/imm_ukf_pda/ukf.cpp
  )
target_link_libraries(imm_ukf_pda
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
add_dependencies(imm_ukf_pda
  ${catkin_EXPORTED_TARGETS}
  )

#visualize_detected_objects
add_executable(visualize_detected_objects
  nodes/visualize_detected_objects/visualize_detected_objects_main.cpp
  nodes/visualize_detected_objects/visualize_detected_objects.cpp
  )
target_link_libraries(visualize_detected_objects
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
add_dependencies(visualize_detected_objects
  ${catkin_EXPORTED_TARGETS}
  )
