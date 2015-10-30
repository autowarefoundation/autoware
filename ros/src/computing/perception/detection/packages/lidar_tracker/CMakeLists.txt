cmake_minimum_required(VERSION 2.8.3)
project(lidar_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  message_generation
  tf
  #vector_map_server
  rosinterface
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   centroids.msg
   geometric_rectangle.msg
   vscan_tracked.msg
   vscan_tracked_array.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
 )

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
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs pcl_ros #vector_map_server
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

#euclidean_clustering node
add_executable(euclidean_cluster nodes/euclidean_cluster/euclidean_cluster.cpp)
#target_link_libraries(euclidean_cluster opencv_highgui opencv_core opencv_contrib ${catkin_LIBRARIES} ${PCL_LIBRARIES} vector_map)
#add_dependencies(euclidean_cluster lidar_tracker_generate_messages_cpp vector_map_server_generate_messages_cpp)
target_link_libraries(euclidean_cluster opencv_highgui opencv_core opencv_contrib opencv_imgproc ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(euclidean_cluster lidar_tracker_generate_messages_cpp)

#vscan_filling node
add_executable(vscan_filling nodes/vscan_filling/vscan_filling.cpp)
target_link_libraries(vscan_filling ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#fusion node
add_executable(obj_fusion nodes/obj_fusion/obj_fusion.cpp)

target_link_libraries(obj_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} m)

add_dependencies(obj_fusion
  lidar_tracker_generate_messages_cpp
  cv_tracker_generate_messages_cpp
)

# build vehicle tracker

execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

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

  set(CUDA_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR/nodes/vehicle_tracker}")
  
  FIND_PACKAGE(CUDA)
  INCLUDE(FindCUDA)
  
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_${CUDA_CAPABILITY_VERSION},code=sm_${CUDA_CAPABILITY_VERSION})
  
  FILE(GLOB SOURCES "*.cu" "*.cpp" "*.c" "*.h")
  
  cuda_add_library(rbsspfvehicletracker
  	${SOURCES})

  catkin_package(
    CATKIN_DEPENDS rosinterface
    )

####
  
    ###

  add_executable(vehicle_tracker
    nodes/vehicle_tracker/main.cpp
    nodes/vehicle_tracker/mainwindow.cpp
#    rbsspfvehicletracker_cuda.o
    )

  set_target_properties(vehicle_tracker
    PROPERTIES COMPILE_FLAGS "-fPIC"
    )

  add_dependencies(vehicle_tracker
    moc_mainwindow
    moc_rbsspfvehicletrackre
    vehicle_tracker_ui_mainwindow
    )

  target_link_libraries(vehicle_tracker
    ${catkin_LIBRARIES}
    rosinterface
#    vehicle_tracker_gpu_parts
    ${Qt5LIBROOT}/libQt5Core.so
    ${Qt5LIBROOT}/libQt5Gui.so
    ${Qt5LIBROOT}/libQt5OpenGL.so
    ${Qt5LIBROOT}/libQt5Widgets.so
    #${CATKIN_DEVEL_PREFIX}/lib/rbsspfvehicletracker_cuda.o
    #rbsspfvehicletracker_cuda.o
    rbsspfvehicletracker
    )


endif()
