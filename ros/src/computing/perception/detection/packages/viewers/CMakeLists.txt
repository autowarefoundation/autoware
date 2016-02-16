cmake_minimum_required(VERSION 2.8.3)
project(viewers)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  runtime_manager
  points2image
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES debug
  CATKIN_DEPENDS std_msgs message_runtime runtime_manager points2image
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(image_viewer nodes/image_viewer/image_viewer.cpp)
target_link_libraries(image_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(image_viewer cv_tracker_generate_messages_cpp)

add_executable(scan_image_viewer nodes/scan_image_viewer/scan_image_viewer.cpp)
target_link_libraries(scan_image_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(scan_image_viewer scan2image_generate_messages_cpp)

add_executable(scan_image_d_viewer nodes/scan_image_d_viewer/scan_image_d_viewer.cpp)
target_link_libraries(scan_image_d_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(scan_image_d_viewer cv_tracker_generate_messages_cpp scan2image_generate_messages_cpp)

add_executable(points_image_viewer nodes/points_image_viewer/points_image_viewer.cpp)
target_link_libraries(points_image_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(points_image_viewer points2image_generate_messages_cpp)

add_executable(image_d_viewer nodes/image_d_viewer/image_d_viewer.cpp)
target_link_libraries(image_d_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(image_d_viewer cv_tracker_generate_messages_cpp)

add_executable(points_image_d_viewer nodes/points_image_d_viewer/points_image_d_viewer.cpp)
target_link_libraries(points_image_d_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(points_image_d_viewer points2image_generate_messages_cpp)

add_executable(vscan_image_viewer nodes/vscan_image_viewer/vscan_image_viewer.cpp)
target_link_libraries(vscan_image_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(vscan_image_viewer points2image_generate_messages_cpp)

add_executable(vscan_image_d_viewer nodes/vscan_image_d_viewer/vscan_image_d_viewer.cpp)
target_link_libraries(vscan_image_d_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(vscan_image_d_viewer points2image_generate_messages_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(traffic_light_viewer nodes/traffic_light_viewer/traffic_light_viewer.cpp)
target_link_libraries(traffic_light_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
add_dependencies(traffic_light_viewer runtime_manager_generate_messages_cpp)
