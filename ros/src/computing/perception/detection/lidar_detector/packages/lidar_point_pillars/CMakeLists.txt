cmake_minimum_required(VERSION 2.8.3)
project(lidar_point_pillars)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  message("CUDA is available!")
  message("CUDA Libs: ${CUDA_LIBRARIES}")
  message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
if(NVINFER AND NVPARSERS)
  message("TensorRT is available!")
  message("NVINFER: ${NVINFER}")
  message("NVPARSERS: ${NVPARSERS}")
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL)

  find_package(autoware_build_flags REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_ros
   autoware_msgs
   )
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

  catkin_package(
   CATKIN_DEPENDS
   roscpp
   pcl_ros
   autoware_msgs
   )

  include_directories(
   include
   ${catkin_INCLUDE_DIRS}
  )
  set(SOURCE_FILES
   nodes/lidar_point_pillars_node.cpp
   nodes/point_pillars_ros.cpp
   )

  add_executable(lidar_point_pillars
         ${SOURCE_FILES})

  add_dependencies(lidar_point_pillars
         ${catkin_EXPORTED_TARGETS}
         )


  cuda_add_library(gpu_point_pillars_lib
         nodes/preprocess_points_cuda.cu
         nodes/anchor_mask_cuda.cu
         nodes/scatter_cuda.cu
         nodes/postprocess_cuda.cu
         nodes/nms_cuda.cu
         )

  target_link_libraries(gpu_point_pillars_lib
   ${CUDA_LIBRARIES}
  )

  add_library(point_pillars_lib
         nodes/point_pillars.cpp
         nodes/preprocess_points.cpp
         )

  target_link_libraries(point_pillars_lib
         nvinfer
         nvonnxparser
         gpu_point_pillars_lib
         )

  target_link_libraries(lidar_point_pillars
         ${catkin_LIBRARIES}
         point_pillars_lib
         )


  install(TARGETS
         gpu_point_pillars_lib
         point_pillars_lib
         lidar_point_pillars
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
         PATTERN ".svn" EXCLUDE)

  install(DIRECTORY include/
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
         PATTERN ".svn" EXCLUDE
         )

  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_gtest(test-point_pillars test/src/test_point_pillars.cpp)
    target_link_libraries(test-point_pillars ${catkin_LIBRARIES} point_pillars_lib)
  endif()
ELSE()
  find_package(catkin REQUIRED)
  catkin_package()
  message("PointPillars won't be built, CUDA and/or TensorRT were not found.")
ENDIF ()
