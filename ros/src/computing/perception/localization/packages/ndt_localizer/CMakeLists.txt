cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)
find_package(PCL REQUIRED)

IF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
SET(FAST_PCL_PACKAGES filters registration)
ENDIF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package( OpenMP )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  runtime_manager
  velodyne_pointcloud
  message_generation
  ${FAST_PCL_PACKAGES}
)

add_message_files(FILES ndt_stat.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS runtime_manager message_runtime std_msgs velodyne_pointcloud ${FAST_PCL_PACKAGES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

IF(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")
ENDIF(PCL_VERSION VERSION_LESS "1.7.2")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
add_executable(lazy_ndt_mapping nodes/lazy_ndt_mapping/lazy_ndt_mapping.cpp)
add_executable(local2global nodes/local2global/local2global.cpp)
add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)

target_link_libraries(ndt_matching ${catkin_LIBRARIES})
target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(lazy_ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(local2global ${catkin_LIBRARIES})
target_link_libraries(queue_counter ${catkin_LIBRARIES})

add_dependencies(ndt_matching runtime_manager_generate_messages_cpp ndt_localizer_generate_messages_cpp)
add_dependencies(ndt_mapping runtime_manager_generate_messages_cpp)
add_dependencies(lazy_ndt_mapping runtime_manager_generate_messages_cpp)
add_dependencies(local2global runtime_manager_generate_messages_cpp)
add_dependencies(queue_counter runtime_manager_generate_messages_cpp)
