cmake_minimum_required(VERSION 2.8.3)
project(pedestrian_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  dpm
  dpm_gpu
  runtime_manager
  fusion
)
find_package(OpenCV REQUIRED)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

###################################
## catkin specific configuration ##
###################################
add_message_files(
  FILES
  FusedObjects.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   CATKIN_DEPENDS car_detector fusion
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)

add_executable(pedestrian_dpm
  nodes/pedestrian_dpm/pedestrian_dpm.cpp
)

set_target_properties(pedestrian_dpm PROPERTIES
  COMPILE_FLAGS "-DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)

target_link_libraries(pedestrian_dpm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  dpm
)

if(EXISTS "/usr/local/cuda")
include_directories(/usr/local/cuda/include)

if("${ARCHITECTURE}" MATCHES "^arm")
LINK_DIRECTORIES(/usr/lib/arm-linux-gnueabihf/tegra)
endif()

add_executable(pedestrian_dpm_gpu
  nodes/pedestrian_dpm_gpu/pedestrian_dpm_gpu.cpp
)

add_dependencies(pedestrian_dpm_gpu runtime_manager_generate_messages_cpp)

SET_TARGET_PROPERTIES(pedestrian_dpm_gpu
  PROPERTIES
  COMPILE_FLAGS "-DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)

target_link_libraries(pedestrian_dpm_gpu
  ${catkin_LIBRARIES}
  cuda
  dpm_gpu
)
else()
message("CUDA is not installed. 'pedestrian_dpm_gpu' is not built.")
endif()

## Declare a cpp executable
add_executable(pedestrian_fusion
  nodes/pedestrian_fusion/pedestrian_fusion.cpp
)

target_link_libraries(pedestrian_fusion
  ${catkin_LIBRARIES} ${OpenCV_LIBS} fusion
)

add_executable(pedestrian_kf
  nodes/pedestrian_kf/pedestrian_kf.cpp
)
target_link_libraries(pedestrian_kf
  ${catkin_LIBRARIES}
  kf
)

add_executable(pedestrian_locate
  nodes/pedestrian_locate/pedestrian_locate.cpp
)
target_link_libraries(pedestrian_locate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

set_target_properties(pedestrian_locate
  PROPERTIES COMPILE_FLAGS
  "-DCAMERA_YAML=${CAMERA_YAML}"
  )
