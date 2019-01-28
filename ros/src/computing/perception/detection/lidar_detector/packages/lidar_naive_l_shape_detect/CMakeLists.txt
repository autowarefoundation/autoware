cmake_minimum_required(VERSION 2.8.3)
project(lidar_naive_l_shape_detect)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  tf
  )

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  autoware_msgs
  tf
  )


link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(
  include
  ${autoware_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(SOURCE_FILES nodes/lidar_naive_l_shape_detect/lidar_naive_l_shape_detect_node.cpp nodes/lidar_naive_l_shape_detect/lidar_naive_l_shape_detect.cpp)

add_executable(lidar_naive_l_shape_detect ${SOURCE_FILES})

add_dependencies(lidar_naive_l_shape_detect
        ${catkin_EXPORTED_TARGETS}
        autoware_msgs_generate_messages_cpp
        )

target_link_libraries(lidar_naive_l_shape_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS
        lidar_naive_l_shape_detect
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
