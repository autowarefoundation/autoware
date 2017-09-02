cmake_minimum_required(VERSION 2.8.3)
project(velodyne_compression)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  pcl_ros
  sensor_msgs
  velodyne_msgs
  pcl_conversions
  velodyne_pointcloud
  )
find_package( OpenCV REQUIRED )


set(${compression}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    velodyne_driver
    velodyne_msgs
    dynamic_reconfigure
)

add_message_files(
  DIRECTORY msg
  FILES
  CompressedPacket.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS system_lib message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(compress2png src/compress2png.cpp)
target_link_libraries(compress2png ${catkin_LIBRARIES})
target_link_libraries(compress2png ${PCL_LIBRARIES})
target_link_libraries(compress2png ${OpenCV_LIBRARIES})

add_executable(recon2packet src/recon2packet.cpp)
target_link_libraries(recon2packet ${catkin_LIBRARIES})
target_link_libraries(recon2packet ${PCL_LIBRARIES})
target_link_libraries(recon2packet ${OpenCV_LIBRARIES})
target_link_libraries(recon2packet ${libpcap_LIBRARIES})

add_dependencies(compress2png velodyne_compression_generate_messages_cpp)
add_dependencies(recon2packet velodyne_compression_generate_messages_cpp)










