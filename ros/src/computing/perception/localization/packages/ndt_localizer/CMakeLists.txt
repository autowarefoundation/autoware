cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  runtime_manager
  velodyne_pointcloud
  message_generation
  filters
  registration
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
  CATKIN_DEPENDS runtime_manager message_runtime std_msgs filters registration
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/fast_pcl/registration/include" "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/fast_pcl/filters/include")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
add_executable(lazy_ndt_mapping nodes/lazy_ndt_mapping/lazy_ndt_mapping.cpp)
add_executable(local2global nodes/local2global/local2global.cpp)
add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)

target_link_libraries(ndt_matching ${catkin_LIBRARIES} fast_pcl_filters fast_pcl_registration)
target_link_libraries(ndt_mapping ${catkin_LIBRARIES} fast_pcl_filters fast_pcl_registration)
target_link_libraries(lazy_ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(local2global ${catkin_LIBRARIES})
target_link_libraries(queue_counter ${catkin_LIBRARIES})

add_dependencies(ndt_matching runtime_manager_generate_messages_cpp ndt_localizer_generate_messages_cpp)
add_dependencies(ndt_mapping runtime_manager_generate_messages_cpp)
add_dependencies(lazy_ndt_mapping runtime_manager_generate_messages_cpp)
add_dependencies(local2global runtime_manager_generate_messages_cpp)
add_dependencies(queue_counter runtime_manager_generate_messages_cpp)
