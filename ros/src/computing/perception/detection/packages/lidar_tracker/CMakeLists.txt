cmake_minimum_required(VERSION 2.8.3)
project(lidar_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  autoware_msgs
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
  rosinterface
  cv_bridge
  vector_map_server
)

pkg_check_modules(Qt5Core REQUIRED Qt5Core)
pkg_check_modules(Qt5Widgets REQUIRED Qt5Widgets)

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

execute_process(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs pcl_ros vector_map_server autoware_msgs 
)

include_directories(
  nodes/euclidean_cluster/includes
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#LidarFuser
#add_executable(lidar_fuser nodes/lidar_fuser/lidar_fuser.cpp)
#target_link_libraries(lidar_fuser opencv_highgui opencv_core opencv_contrib opencv_imgproc ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(lidar_fuser lidar_tracker_generate_messages_cpp)

#SVM Detect
add_executable(svm_lidar_detect nodes/svm_lidar_detect/svm_lidar_detect.cpp)
#add_library(svm SHARED nodes/svm_lidar_detect/libsvm/svm.cpp)
target_link_libraries (svm_lidar_detect PRIVATE
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES} 
#	svm
	${OpenCV_LIBRARIES}	
)

#target_include_directories(svm_lidar_detect PRIVATE nodes/svm_lidar_detect/libsvm)
add_dependencies(svm_lidar_detect ${catkin_EXPORTED_TARGETS})

#ParticleFilter Track
add_executable(pf_lidar_track nodes/pf_lidar_track/pf_lidar_track.cpp)
target_link_libraries(pf_lidar_track ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pf_lidar_track ${catkin_EXPORTED_TARGETS})

#KalmanFilter Track
add_executable(kf_lidar_track 
				nodes/kf_lidar_track/kf_lidar_track.cpp 
				nodes/kf_lidar_track/KfLidarTracker.cpp 
				nodes/kf_lidar_track/HungarianAlg.cpp
				nodes/kf_lidar_track/Kalman.cpp
				nodes/euclidean_cluster/Cluster.cpp
				)
target_include_directories(kf_lidar_track PRIVATE nodes/kf_lidar_track/includes)
target_link_libraries(kf_lidar_track ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(kf_lidar_track ${catkin_EXPORTED_TARGETS})

#Euclidean Track
add_executable(euclidean_lidar_track nodes/euclidean_lidar_track/euclidean_lidar_track.cpp)
target_link_libraries(euclidean_lidar_track ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(euclidean_lidar_track ${catkin_EXPORTED_TARGETS})

#Vscan Filling
add_executable(vscan_filling nodes/vscan_filling/vscan_filling.cpp)
target_link_libraries(vscan_filling ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#Object Fusion
add_executable(obj_fusion nodes/obj_fusion/obj_fusion.cpp)
target_link_libraries(obj_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} m)
add_dependencies(obj_fusion ${catkin_EXPORTED_TARGETS})

# Vscan Track
execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

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

  add_custom_target(gpu_arch_version_checker)
  add_dependencies(gpu_arch_version_checker
    libdpm_ttic
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

  #QT Stuff
  # set(CMAKE_AUTOMOC ON)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  add_custom_command(
    OUTPUT ui_mainwindow.h
    COMMAND
    ${Qt5BIN}/uic
    "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vscan_lidar_track/ui_mainwindow.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vscan_lidar_track/mainwindow.ui"
    )
  add_custom_target(vehicle_tracker_ui_mainwindow DEPENDS ui_mainwindow.h)

  add_custom_command(
    OUTPUT moc_mainwindow.cpp
    COMMAND
    ${Qt5BIN}/moc
    "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vscan_lidar_track/moc_mainwindow.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vscan_lidar_track/mainwindow.h"
    )
  add_custom_target(vehicle_tracker_moc_mainwindow DEPENDS moc_mainwindow.cpp)

  add_custom_command(
    OUTPUT moc_rbsspfvehicletracker.cpp
    COMMAND
    ${Qt5BIN}/moc
    "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vscan_lidar_track/moc_rbsspfvehicletracker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vscan_lidar_track/rbsspfvehicletracker.h"
    )
  add_custom_target(vehicle_tracker_moc_rbsspfvehicletracker DEPENDS moc_rbsspfvehicletracker.cpp)

  FIND_PACKAGE(CUDA REQUIRED)
  INCLUDE(FindCUDA)

  if ("${ROS_VERSION}" MATCHES "(kinetic)")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH})
  else()
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11)
  endif()

  INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS})

  #cuda pf lib
  cuda_add_library(rbsspfvehicletracker
        nodes/vscan_lidar_track/rbsspfvehicletracker.cu
        nodes/vscan_lidar_track/rbsspfvehicletracker.cuh)

  catkin_package(
    CATKIN_DEPENDS rosinterface
    )

  #qt exe
  include_directories(${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    nodes/vscan_lidar_track/)

  add_definitions(${Qt5Core_DEFINITIONS})
  add_definitions(${Qt5Widgets_DEFINITIONS})

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Core_EXECUTABLE_COMPILE_FLAGS}")

  add_executable(vscan_lidar_track
    nodes/vscan_lidar_track/rbsspfvehicletracker.h
    nodes/vscan_lidar_track/main.cpp
    nodes/vscan_lidar_track/mainwindow.cpp
    nodes/vscan_lidar_track/rbsspfvehicletracker.cpp
    )

  set_target_properties(vscan_lidar_track
    PROPERTIES COMPILE_FLAGS "-fPIC"
    )

  add_dependencies(vscan_lidar_track
    vehicle_tracker_moc_mainwindow
    vehicle_tracker_moc_rbsspfvehicletracker
    vehicle_tracker_ui_mainwindow
    ${catkin_EXPORTED_TARGETS}
    )

  target_link_libraries(vscan_lidar_track
    ${catkin_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    rbsspfvehicletracker
    )


endif()

#Euclidean Cluster
add_executable(euclidean_cluster nodes/euclidean_cluster/euclidean_cluster.cpp nodes/euclidean_cluster/Cluster.cpp)

find_package(CUDA)
if(${CUDA_FOUND})
	INCLUDE(FindCUDA)
	message("-- USING ACCELERATED CLUSTERING --")
	message("Version: " ${CUDA_VERSION})
	message("Library: " ${CUDA_CUDA_LIBRARY})
	message("Runtime: " ${CUDA_CUDART_LIBRARY})
	
	target_compile_definitions(euclidean_cluster PRIVATE
		GPU_CLUSTERING=1
	)

	cuda_add_library(gpu_euclidean_clustering
		nodes/euclidean_cluster/includes/gpu_euclidean_clustering.h
		nodes/euclidean_cluster/gpu_euclidean_clustering.cu
	)
	
	target_link_libraries(euclidean_cluster 
		${OpenCV_LIBRARIES}
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		gpu_euclidean_clustering)
	
else()
	target_link_libraries(euclidean_cluster ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
	
endif()

add_dependencies(euclidean_cluster lidar_tracker_generate_messages_cpp vector_map_server_generate_messages_cpp)
