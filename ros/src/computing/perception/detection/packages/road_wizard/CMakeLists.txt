cmake_minimum_required(VERSION 2.8.3)
project(road_wizard)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  tf
  libvectormap
  runtime_manager
  vector_map_server
  geometry_msgs
  waypoint_follower
  cmake_modules
  autoware_msgs
  )
find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

pkg_check_modules(Qt5Core REQUIRED Qt5Core)
pkg_check_modules(Qt5Widgets REQUIRED Qt5Widgets)


###################################
## catkin specific configuration ##
###################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS std_msgs message_runtime libvectormap autoware_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########
add_library(libcontext lib/Context.cpp)
add_dependencies(libcontext
  ${catkin_EXPORTED_TARGETS})


### region_tlr ###
add_executable(region_tlr
  nodes/region_tlr/region_tlr.cpp
  nodes/region_tlr/TrafficLightDetector.cpp
  )

target_link_libraries(region_tlr
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  libcontext
)

add_dependencies(region_tlr
  ${catkin_EXPORTED_TARGETS}
)

### feat_proj ###
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${libvectormap_INCLUDE_DIRS}
)

add_executable(feat_proj
  nodes/feat_proj/feat_proj.cpp
)

target_link_libraries(feat_proj
  ${catkin_LIBRARIES}
  ${libvectormap_LIBRARIES}
  ${OpenCV_LIBS}
  vector_map
)

add_dependencies(feat_proj
<<<<<<< HEAD
  ${catkin_EXPORTED_TARGETS}
=======
runtime_manager_generate_messages_cpp
road_wizard_generate_messages_cpp
vector_map_server_generate_messages_cpp
geometry_msgs_generate_messages_cpp
waypoint_follower_generate_messages_cpp
>>>>>>> fe3055867d3a8cd8990ab0541b7a69538eb1c322
)

### tlr_tuner ###
EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  nodes/tlr_tuner/
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_custom_command(
  OUTPUT ui_mainwindow.h
  COMMAND
  ${Qt5BIN}/uic
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/ui_mainwindow.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/mainwindow.ui"
)
add_custom_target(tlr_tuner_ui_mainwindow DEPENDS ui_mainwindow.h)

add_custom_command(
  OUTPUT mainwindow_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/mainwindow_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/mainwindow.h"
)
add_custom_target(tlr_tuner_mainwindow_moc DEPENDS mainwindow_moc.cpp)

add_executable(tlr_tuner
  nodes/tlr_tuner/tlr_tuner.cpp
  nodes/tlr_tuner/mainwindow.cpp
  nodes/tlr_tuner/tunerBody.cpp
  )

set_target_properties(tlr_tuner
  PROPERTIES COMPILE_FLAGS "-fPIC"
  )

add_dependencies(tlr_tuner
  tlr_tuner_ui_mainwindow
  tlr_tuner_mainwindow_moc
  )

target_link_libraries(tlr_tuner
  ${catkin_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${OpenCV_LIBS}
  )


### roi_extractor ###
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(roi_extractor
  nodes/roi_extractor/roi_extractor.cpp
  )

target_link_libraries(roi_extractor
  ${catkin_LIBRARIES}
  libcontext
)

add_dependencies(region_tlr
road_wizard_generate_messages_cpp)


### label_maker ###
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(TinyXML REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  nodes/label_maker/
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

file(GLOB SOURCES "nodes/label_maker/*.cpp")
file(GLOB HEADERS "nodes/label_maker/*.h")
file(GLOB UI_SOURCES "nodes/label_maker/*.ui")

qt5_wrap_ui(UI_HEADERS ${UI_SOURCES})

add_executable(label_maker
  ${SOURCES}
  ${HEADERS}
  ${UI_HEADERS}
  )

set_target_properties(label_maker
  PROPERTIES COMPILE_FLAGS "-fPIC"
  )

target_link_libraries(label_maker
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  )


### region_tlr_ssd ###
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(SSD_CAFFE_PATH "$ENV{HOME}/ssdcaffe/distribute")
####################################################################################################
if(EXISTS "${SSD_CAFFE_PATH}")

find_package(CUDA)
pkg_check_modules(HDF5 hdf5)

if(HDF5_FOUND)
  include_directories(${HDF5_INCLUDE_DIRS})
endif()

add_executable(region_tlr_ssd
  nodes/region_tlr_ssd/region_tlr_ssd.cpp
  nodes/region_tlr_ssd/traffic_light_recognizer.cpp
  )

target_link_libraries(region_tlr_ssd
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${CUDA_CUBLAS_LIBRARIES}
  ${CUDA_curand_LIBRARY}
  ${SSD_CAFFE_PATH}/lib/libcaffe.so
  glog
  libcontext
  )

target_include_directories(region_tlr_ssd PRIVATE
  ${SSD_CAFFE_PATH}/include
  ${CUDA_INCLUDE_DIRS}
  nodes/region_tlr_ssd/
  )

add_dependencies(region_tlr_ssd
  cv_tracker_generate_messages_cpp
  runtime_manager_generate_messages_cpp
  road_wizard_generate_messages_cpp
  )

else()                          # if(EXISTS "${SSD_CAFFE_PATH}")
  message("'SSD/Caffe' is not installed. 'region_tlr_ssd' will not be built.")
endif()                         # if(EXISTS "${SSD_CAFFE_PATH}")
