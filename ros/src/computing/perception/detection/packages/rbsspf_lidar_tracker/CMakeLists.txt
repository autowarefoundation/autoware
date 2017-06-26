cmake_minimum_required(VERSION 2.8.3)
project(rbsspf_lidar_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  message_filters  
  message_generation
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
)

find_package(Boost REQUIRED COMPONENTS 
  signals
) 

execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if ("${ROS_VERSION}" MATCHES "(kinetic)")
	pkg_check_modules(OpenCV REQUIRED opencv-3.2.0-dev)
else()
	pkg_check_modules(OpenCV REQUIRED)
endif()

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs pcl_ros 
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

###################
###CUDA CHECKER####
###ARCH CHECKER####
if(EXISTS "/usr/local/cuda")
  include_directories(
    ${catkin_INCLUDE_DIRS}
    "/usr/local/cuda/include"
    )

  if("${ARCHITECTURE}" MATCHES "^arm")
    LINK_DIRECTORIES(/usr/lib/arm-linux-gnueabihf/tegra)
  endif()

  set(CUDA_COMPILER "/usr/local/cuda/bin/nvcc")

  if("${ARCHITECTURE}" MATCHES "^arm")
    set(CUDA_LDFLAGS -L/usr/lib/arm-linux-gnueabihf/tegra -lcuda)
  else()
    set(CUDA_LDFLAGS -lcuda)
  endif()

  set(CUDA_CAPABILITY_VERSION_CHECKER
    "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker"
    )

  execute_process(
    COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
    OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

  if("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
    set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
  else()
    set(CUDA_ARCH "sm_52")
  endif()
  
  FIND_PACKAGE(CUDA REQUIRED)
  INCLUDE(FindCUDA)

  if ("${ROS_VERSION}" MATCHES "(kinetic)")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH})
  else()
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -arch=${CUDA_ARCH}; -std=c++11)
  endif()
  
  message("---------------------------")
  message("-- Architecture: " ${CUDA_ARCH})
  message("-- Version:      " ${CUDA_VERSION})
  message("-- Library:      " ${CUDA_CUDA_LIBRARY})
  message("-- Runtime:      " ${CUDA_CUDART_LIBRARY})
  message("-- NVCC FLAGS:   " ${CUDA_NVCC_FLAGS})

endif()
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

cuda_add_executable(rbsspf_lidar_tracker
  src/rbsspf_lidar_tracker.cpp
)

set(CUDA_SEPARABLE_COMPILATION ON)

cuda_add_library(rbsspfvehicletracker_lib
#  src/rbsspfvehicletracker.cu
#  src/rbsspfvehicletracker.cuh
  src/rbsspf_share.cu
  src/rbsspf_share.cuh
  src/rbsspf_geometry.cu
  src/rbsspf_geometry.cuh
  src/rbsspf_motion.cu
  src/rbsspf_motion.cuh
  src/rbsspf_tracker.cu
  src/rbsspf_tracker.cuh
  OPTIONS --compiler-options "-fPIC"
)

target_link_libraries(rbsspf_lidar_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${CUDA_CUBLAS_LIBRARIES}
  ${CUDA_curand_LIBRARY}
  ${Boost_LIBRARIES}
  rbsspfvehicletracker_lib
)

target_include_directories(rbsspf_lidar_tracker PRIVATE
  include/rbsspf_lidar_tracker
  ${Boost_INCLUDE_DIRS}
)

