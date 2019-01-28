cmake_minimum_required(VERSION 2.8.3)
project(viewers)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        cv_bridge
        image_transport
        )

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs autoware_msgs cv_bridge image_transport
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")


add_executable(image_viewer nodes/image_viewer/image_viewer.cpp)
target_link_libraries(image_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(image_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

add_executable(scan_image_viewer nodes/scan_image_viewer/scan_image_viewer.cpp)
target_link_libraries(scan_image_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(scan_image_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

add_executable(scan_image_d_viewer nodes/scan_image_d_viewer/scan_image_d_viewer.cpp)
target_link_libraries(scan_image_d_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(scan_image_d_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

add_executable(points_image_viewer nodes/points_image_viewer/points_image_viewer.cpp)
target_link_libraries(points_image_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(points_image_viewer  
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

add_executable(image_d_viewer nodes/image_d_viewer/image_d_viewer.cpp)
target_link_libraries(image_d_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(image_d_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

add_executable(points_image_d_viewer nodes/points_image_d_viewer/points_image_d_viewer.cpp)
target_link_libraries(points_image_d_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(points_image_d_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

add_executable(vscan_image_viewer nodes/vscan_image_viewer/vscan_image_viewer.cpp)
target_link_libraries(vscan_image_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(vscan_image_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

add_executable(vscan_image_d_viewer nodes/vscan_image_d_viewer/vscan_image_d_viewer.cpp)
target_link_libraries(vscan_image_d_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(vscan_image_d_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        include
)
add_executable(traffic_light_viewer nodes/traffic_light_viewer/traffic_light_viewer.cpp)
target_link_libraries(traffic_light_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )
add_dependencies(traffic_light_viewer 
  ${catkin_EXPORTED_TARGETS}
  autoware_msgs_generate_messages_cpp
  )

install(TARGETS
        image_viewer
        traffic_light_viewer
        vscan_image_d_viewer
        points_image_d_viewer
        image_d_viewer
        scan_image_d_viewer
        scan_image_viewer
        image_viewer

        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
