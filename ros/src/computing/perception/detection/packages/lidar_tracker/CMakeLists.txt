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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
## Declare a cpp library
# add_library(lidar_tracker
#   src/${PROJECT_NAME}/lidar_tracker.cpp
# )

## Declare a cpp executable
# add_executable(lidar_tracker_node src/lidar_tracker_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(lidar_tracker_node lidar_tracker_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(lidar_tracker_node
#   ${catkin_LIBRARIES}
# )

add_executable(euclidean_cluster nodes/euclidean_cluster/euclidean_cluster.cpp)
#target_link_libraries(euclidean_cluster opencv_highgui opencv_core opencv_contrib ${catkin_LIBRARIES} ${PCL_LIBRARIES} vector_map)
#add_dependencies(euclidean_cluster lidar_tracker_generate_messages_cpp vector_map_server_generate_messages_cpp)
target_link_libraries(euclidean_cluster opencv_highgui opencv_core opencv_contrib opencv_imgproc ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(euclidean_cluster lidar_tracker_generate_messages_cpp)

add_executable(vscan_filling nodes/vscan_filling/vscan_filling.cpp)
target_link_libraries(vscan_filling ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(obj_fusion
  nodes/obj_fusion/obj_fusion.cpp
)

target_link_libraries(obj_fusion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  m
)

add_dependencies(obj_fusion
  lidar_tracker_generate_messages_cpp
  cv_tracker_generate_messages_cpp
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lidar_tracker lidar_tracker_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# build vehicle tracker

execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

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
  
  FILE(GLOB SOURCES "*.cu" "*.cpp" "*.c" "*.h")
  
  cuda_add_executable(rbsspfvehicletracker
  	${SOURCES})
  
  LIST(APPEND CMAKE_CXX_FLAGS "-std=c++11 -rdc=true --compiler-options -fPIC -m64 -O3")
  LIST(APPEND CUDA_NVCC_FLAGS "-arch=${CUDA_ARCH} -c --compiler-options -fno-strict-aliasing")
  LIST(APPEND CUDA_NVCC_FLAGS "-use_fast_math --ptxas-options=-v")
  
  target_link_libraries(rbsspfvehicletracker
    /usr/local/cuda/lib/libcudart)

  #add_custom_command(
  #  OUTPUT rbsspfvehicletracker_cuda.o
  #  COMMAND ${CUDA_COMPILER}
  #  "-std=c++11" "-rdc=true" "--compiler-options" "-fPIC" "-m64" "-O3"
  #  "-arch=${CUDA_ARCH}" "-c" "--compiler-options" "-fno-strict-aliasing"
  #  "-use_fast_math" "--ptxas-options=-v"
  #  "-I/usr/local/cuda/include"  "-L/usr/local/cuda/lib64"
  #  "-lcudart" "-lcuda" "-lcudadevrt"
  #  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/rbsspfvehicletracker.cu"
  #  "-o" "${CATKIN_DEVEL_PREFIX}/lib/rbsspfvehicletracker_cuda.o"
  #  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/rbsspfvehicletracker.cu"
  #  )
    
  # add_library(vehicle_tracker_gpu_parts
  #   STATIC
  #   rbsspfvehicletracker_cuda.o
  #   )
  # set_source_files_properties(rbsspfvehicletracker_cuda.o
  #   PROPERTIES
  #   EXTERNAL_OBJECT true
  #   GENERATED true
  #   )
  # set_target_properties(vehicle_tracker_gpu_parts
  #   PROPERTIES
  #   LINKER_LANGUAGE CUDA
  #   )

  catkin_package(
    CATKIN_DEPENDS rosinterface
    )

  if(EXISTS /usr/local/qtbase5)
    # Self install version
    set(Qt5INCLUDE "/usr/local/qtbase5/include")
    set(Qt5BIN "/usr/local/qtbase5/bin")
    set(Qt5LIBROOT "/usr/local/qtbase5/lib")
  else()
    # Ubuntu Qt5 package
    set(Qt5INCLUDE "/usr/include/qt5")
    set(Qt5BIN "/usr/lib/x86_64-linux-gnu/qt5/bin")
    set(Qt5LIBROOT "/usr/lib/x86_64-linux-gnu")
  endif()

  include_directories(
    ${catkin_INCLUDE_DIRS}
    nodes/vehicle_tracker
    ${Qt5INCLUDE}
    ${Qt5INCLUDE}/QtCore
    ${Qt5INCLUDE}/QtGui
    ${Qt5INCLUDE}/QtOpenGL
    ${Qt5INCLUDE}/QtWidgets
    )

  add_custom_command(
    OUTPUT moc_rbsspfvehicletracker.cpp
    COMMAND ${Qt5BIN}/moc
    "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/moc_rbsspfvehicletracker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/moc_rbsspfvehicletracker.h"
    )
  add_custom_target(moc_rbsspfvehicletracker
    DEPENDS moc_rbsspfvehicletracker.cpp
    )

  add_custom_command(
    OUTPUT ui_mainwindow.h
    COMMAND ${Qt5BIN}/uic
    "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/ui_mainwindow.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/mainwindow.ui"
    )
  add_custom_target(vehicle_tracker_ui_mainwindow
    DEPENDS ui_mainwindow.h
    )

  add_custom_command(
    OUTPUT moc_mainwindow.cpp
    COMMAND ${Qt5BIN}/moc
    "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/moc_mainwindow.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_tracker/mainwindow.h"
    )
  add_custom_target(moc_mainwindow
    DEPENDS moc_mainwindow.cpp
    )

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
    )


endif()
