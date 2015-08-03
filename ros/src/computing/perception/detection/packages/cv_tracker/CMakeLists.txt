cmake_minimum_required(VERSION 2.8.3)
project(cv_tracker)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  libdpm_ttic
  fusion
  kf
)
FIND_PACKAGE(OpenCV REQUIRED)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_message_files(
  FILES
  image_obj.msg
  image_obj_ranged.msg
  image_obj_tracked.msg
  image_rect.msg
  image_rect_ranged.msg
  obj_pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS libdpm_ttic scan2image fusion kf
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -DROS ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)

IF(EXISTS "/usr/local/cuda")
INCLUDE_DIRECTORIES(/usr/local/cuda/include)

IF("${ARCHITECTURE}" MATCHES "^arm")
LINK_DIRECTORIES(/usr/lib/arm-linux-gnueabihf/tegra)
endif()

ELSE()
message("CUDA is not installed. 'cv_tracker' package is not built.")
ENDIF()

## dpm_ttic ##
ADD_EXECUTABLE(dpm_ttic
  nodes/dpm_ttic/dpm_ttic.cpp
)

TARGET_LINK_LIBRARIES(dpm_ttic
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  libdpm_ttic
)

SET_TARGET_PROPERTIES(dpm_ttic
  PROPERTIES
  COMPILE_FLAGS
  "-DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)

ADD_DEPENDENCIES(dpm_ttic
  cv_tracker_generate_messages_cpp
)

SET(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

#SET_TARGET_PROPERTIES(car_locate
#  PROPERTIES COMPILE_FLAGS
#  "-DCAMERA_YAML=${CAMERA_YAML}")

## range_fusion ##
ADD_EXECUTABLE(range_fusion
  nodes/range_fusion/range_fusion.cpp
)

TARGET_LINK_LIBRARIES(range_fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  fusion
)

ADD_DEPENDENCIES(range_fusion
  cv_tracker_generate_messages_cpp
)

## kf_tracker ##
ADD_EXECUTABLE(kf_track
  nodes/kf_track/kf_track.cpp
)

TARGET_LINK_LIBRARIES(kf_track
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  kf
)

ADD_DEPENDENCIES(kf_track
  kf_track_generate_messages_cpp
)

## obj_reproj ##
ADD_EXECUTABLE(obj_reproj
  nodes/obj_reproj/obj_reproj.cpp
)

TARGET_LINK_LIBRARIES(obj_reproj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

ADD_DEPENDENCIES(obj_reproj
  obj_reproj_generate_messages_cpp
)

## kf_tracker ##
ADD_EXECUTABLE(klt_track
  nodes/klt_track/klt_track.cpp
)

TARGET_LINK_LIBRARIES(klt_track
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  klt
)

ADD_DEPENDENCIES(klt_track
  klt_track_generate_messages_cpp
)