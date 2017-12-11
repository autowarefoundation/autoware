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

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

###################################
## catkin specific configuration ##
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS std_msgs message_runtime geometry_msgs vector_map_server libvectormap autoware_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########
add_library(libcontext lib/Context.cpp)
add_dependencies(libcontext
  ${catkin_EXPORTED_TARGETS}
)


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
  ${catkin_EXPORTED_TARGETS}
)

### tlr_tuner ###
EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  include
  nodes/tlr_tuner/
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

qt5_wrap_ui(tlr_tuner_ui_mainwindow nodes/tlr_tuner/mainwindow.ui)

add_executable(tlr_tuner
  nodes/tlr_tuner/tlr_tuner.cpp
  nodes/tlr_tuner/mainwindow.cpp
  nodes/tlr_tuner/mainwindow.h
  nodes/tlr_tuner/tunerBody.cpp
  nodes/tlr_tuner/tunerBody.h
  ${tlr_tuner_ui_mainwindow}
  )

set_target_properties(tlr_tuner
  PROPERTIES COMPILE_FLAGS "-fPIC"
  )

target_link_libraries(tlr_tuner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Qt5::Core
  Qt5::Widgets
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
  ${catkin_EXPORTED_TARGETS}
  )

else()                          # if(EXISTS "${SSD_CAFFE_PATH}")
  message("'SSD/Caffe' is not installed. 'region_tlr_ssd' will not be built.")
endif()                         # if(EXISTS "${SSD_CAFFE_PATH}")
