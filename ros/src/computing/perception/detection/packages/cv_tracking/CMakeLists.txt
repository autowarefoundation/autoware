cmake_minimum_required(VERSION 2.8.3)
project(cv_tracking)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  dpm
  dpm_gpu
  libdpm_ttic
  fusion
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
#  INCLUDE_DIRS include
  CATKIN_DEPENDS dpm dpm_gpu libdpm_ttic scan2image fusion
#  DEPENDS system_lib
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
message("CUDA is not installed. 'cv_tracking' package is not built.")
ENDIF()

## Nodes
##  - [ ] dpm_ocv
##  - [ ] dpm_ttic
##  - [ ] fusion_ranging
##  - [ ] obj_reproj
##  - [ ] kf_tracking

ADD_EXECUTABLE(dpm_ttic
  nodes/dpm_ttic/dpm_ttic.cpp
)

TARGET_LINK_LIBRARIES(dpm_ttic
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ##dpm
  dpm_gpu
  libdpm_ttic
)

SET_TARGET_PROPERTIES(dpm_ttic
  PROPERTIES
  COMPILE_FLAGS
  "-DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)

ADD_DEPENDENCIES(dpm_ttic
  dpm_ttic_generate_messages_cpp
)

SET(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

#SET_TARGET_PROPERTIES(car_locate
#  PROPERTIES COMPILE_FLAGS
#  "-DCAMERA_YAML=${CAMERA_YAML}")
