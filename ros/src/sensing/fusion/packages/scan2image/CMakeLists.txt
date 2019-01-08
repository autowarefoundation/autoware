cmake_minimum_required(VERSION 2.8.3)
project(scan2image)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        )
find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs autoware_msgs message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(scan2image nodes/scan2image/scan2image.cpp)

set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")
set(MANUAL_YAML "${CMAKE_CURRENT_SOURCE_DIR}/manual.yaml")

set_target_properties(scan2image
        PROPERTIES COMPILE_FLAGS
        "-DCAMERA_YAML=${CAMERA_YAML} -DMANUAL_YAML=${MANUAL_YAML}")

target_link_libraries(scan2image ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(scan2image
        ${catkin_EXPORTED_TARGETS}
        )
