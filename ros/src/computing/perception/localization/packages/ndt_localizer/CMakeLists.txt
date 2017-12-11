cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

IF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
SET(FAST_PCL_PACKAGES filters registration)
ENDIF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package( OpenMP )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND FAST_PCL_PACKAGES ndt_gpu)
endif()

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  autoware_msgs
  pcl_conversions
  velodyne_pointcloud
  ${FAST_PCL_PACKAGES}
  ndt_tku
  ndt_cpu
)
elseif("${ROS_VERSION}" MATCHES "(kinetic)")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  autoware_msgs
  pcl_conversions
  velodyne_pointcloud
  ${FAST_PCL_PACKAGES}
  ndt_cpu
)
endif()

###################################
## catkin specific configuration ##
###################################
if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs ${FAST_PCL_PACKAGES}
  DEPENDS ndt_tku ndt_cpu
#  DEPENDS system_lib
)
elseif("${ROS_VERSION}" MATCHES "(kinetic)")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs ${FAST_PCL_PACKAGES}
  DEPENDS ndt_cpu
#  DEPENDS ndt_tku
#  DEPENDS system_lib
)
endif()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
add_dependencies(ndt_matching autoware_msgs_generate_messages_cpp)
add_dependencies(ndt_mapping autoware_msgs_generate_messages_cpp)

if(CUDA_FOUND)
  target_include_directories(ndt_matching PRIVATE ${CUDA_INCLUDE_DIRS})
  target_include_directories(ndt_mapping PRIVATE ${CUDA_INCLUDE_DIRS})
endif()


if(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
  add_executable(ndt_matching_omp nodes/ndt_matching/ndt_matching.cpp)
  add_executable(ndt_mapping_omp nodes/ndt_mapping/ndt_mapping.cpp)
  target_link_libraries(ndt_matching_omp ${catkin_LIBRARIES})
  target_link_libraries(ndt_mapping_omp ${catkin_LIBRARIES})
  add_dependencies(ndt_matching_omp autoware_msgs_generate_messages_cpp)
  add_dependencies(ndt_mapping_omp autoware_msgs_generate_messages_cpp)

  set_target_properties(ndt_matching_omp  PROPERTIES COMPILE_DEFINITIONS "USE_FAST_PCL")
  set_target_properties(ndt_mapping_omp   PROPERTIES COMPILE_DEFINITIONS "USE_FAST_PCL")

  if(CUDA_FOUND)
    target_include_directories(ndt_matching_omp PRIVATE ${CUDA_INCLUDE_DIRS})
    target_include_directories(ndt_mapping_omp PRIVATE ${CUDA_INCLUDE_DIRS})
  endif()

endif(NOT (PCL_VERSION VERSION_LESS "1.7.2"))


add_executable(approximate_ndt_mapping nodes/approximate_ndt_mapping/approximate_ndt_mapping.cpp)
add_executable(tf_mapping nodes/tf_mapping/tf_mapping.cpp)
add_executable(lazy_ndt_mapping nodes/lazy_ndt_mapping/lazy_ndt_mapping.cpp)
add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)
target_link_libraries(approximate_ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(tf_mapping ${catkin_LIBRARIES})
target_link_libraries(lazy_ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(queue_counter ${catkin_LIBRARIES})
add_dependencies(lazy_ndt_mapping autoware_msgs_generate_messages_cpp)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
  #add_executable(ndt_matching_tku nodes/ndt_matching_tku/ndt_matching_tku.cpp nodes/ndt_matching_tku/newton.cpp nodes/ndt_matching_tku/algebra.cpp)
  add_executable(ndt_matching_tku nodes/ndt_matching_tku/ndt_matching_tku.cpp)
  add_executable(ndt_mapping_tku nodes/ndt_mapping_tku/ndt_mapping_tku.cpp)
  add_executable(mapping nodes/ndt_mapping_tku/mapping.cpp)
  target_link_libraries(ndt_matching_tku ndt_tku ${catkin_LIBRARIES})
  target_link_libraries(ndt_mapping_tku ndt_tku ${catkin_LIBRARIES})
  target_link_libraries(mapping ndt_tku ${catkin_LIBRARIES})
endif()
