cmake_minimum_required(VERSION 2.8.3)
project(lidar_svm_detect)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  sensor_msgs
  autoware_msgs
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
  )

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  autoware_msgs
  tf
  jsk_recognition_msgs
  jsk_rviz_plugins
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(${OpenCV_LIBRARY_DIRS})

#SVM Detect
add_executable(lidar_svm_detect
  nodes/lidar_svm_detect/lidar_svm_detect.cpp)
#add_library(svm SHARED nodes/svm_lidar_detect/libsvm/svm.cpp)
target_link_libraries(lidar_svm_detect PRIVATE
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  #	svm
  ${OpenCV_LIBRARIES}
  )

#target_include_directories(svm_lidar_detect PRIVATE nodes/svm_lidar_detect/libsvm)
add_dependencies(lidar_svm_detect ${catkin_EXPORTED_TARGETS})

install(TARGETS
        lidar_svm_detect
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
