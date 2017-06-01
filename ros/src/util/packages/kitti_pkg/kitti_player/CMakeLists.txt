cmake_minimum_required(VERSION 2.8.3)
project(kitti_player)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")

#SET (CMAKE_BUILD_TYPE Debug)
#SET (ROS_BUILD_TYPE Debug)
SET (CMAKE_BUILD_TYPE Release)
SET (ROS_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "-O3 -std=c++11")


find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    tf
                    tf2
                    std_msgs
                    pcl_ros
                    geometry_msgs
                    cv_bridge
                    image_transport
)

find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(cfg/kitti_player.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

include_directories(
		${}
  		${catkin_INCLUDE_DIRS} 
  		${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(kitti_player src/kitti_player.cpp)

target_link_libraries(kitti_player ${catkin_LIBRARIES} ${PCL_LIBRARIES})

endif()
