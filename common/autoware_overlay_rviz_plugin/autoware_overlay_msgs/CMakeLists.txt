cmake_minimum_required(VERSION 3.5)
project(autoware_overlay_msgs)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/OverlayText.msg"
        DEPENDENCIES
        std_msgs
)

ament_package()
