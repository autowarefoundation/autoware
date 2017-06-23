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
  message_generation
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
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
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11)
  endif()

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

ADD_EXECUTABLE(rbsspf_lidar_tracker
  src/rbsspf_lidar_tracker.cpp
)

TARGET_LINK_LIBRARIES(rbsspf_lidar_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${CUDA_CUBLAS_LIBRARIES}
  ${CUDA_curand_LIBRARY}
)

TARGET_INCLUDE_DIRECTORIES( rbsspf_lidar_tracker PRIVATE
  include/rbsspf_lidar_tracker
)

