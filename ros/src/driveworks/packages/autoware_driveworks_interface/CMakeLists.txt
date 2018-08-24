cmake_minimum_required(VERSION 2.8.3)
project(autoware_driveworks_interface)

set(CMAKE_CXX_FLAGS "-O2 -s  -Wno-error=unused-parameter")


find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs sensor_msgs autoware_msgs
        )

find_package(CUDA)

catkin_package(
        CATKIN_DEPENDS std_msgs sensor_msgs autoware_msgs
)

if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64" AND CUDA_FOUND)
    find_package(Threads REQUIRED)
    find_package(OpenCV REQUIRED)

    find_package(catkin REQUIRED COMPONENTS
            roscpp
            cv_bridge
            std_msgs
            sensor_msgs
            autoware_msgs
            tf
            )

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
    )

    link_directories(
            ${catkin_LIBRARY_DIRS}
            ${PCL_LIBRARY_DIRS}
            ${OpenCV_LIBRARY_DIRS}
            /usr/lib/
            lib
            /usr/local/driveworks/lib/
            ${DRIVEWORKS_TOOLKIT_TARGET_DIR}/lib
            ${CUDA_TOOLKIT_TARGET_DIR}/lib
            ${CUDA_TOOLKIT_TARGET_DIR}/lib/stubs
            ${NVIDIA_DRIVE_SDK_TARGET_DIR}/usr/lib
            ${NVIDIA_TENSORRT_TARGET_DIR}/lib
    )

    add_executable(gmsl_interface
            nodes/gmsl_interface/gmsl_interface_node.cpp
            )

    add_dependencies(gmsl_interface
            ${catkin_EXPORTED_TARGETS}
            )

    target_link_libraries(gmsl_interface
            ${catkin_LIBRARIES}
            autoware_driveworks
            driveworks
            cudart
            cuda
            )

    add_executable(tensorrt_interface
            nodes/tensorrt_interface/tensorrt_interface_node.cpp
            )

    add_dependencies(tensorrt_interface
            ${catkin_EXPORTED_TARGETS}
            )

    target_link_libraries(tensorrt_interface
            ${catkin_LIBRARIES}
            autoware_driveworks
            driveworks
            cudart
            cuda
            )
     # Use configure_file instead of file(COPY ...) so that if the input file
     # changes, it gets copied to the DEVEL space anyway
     configure_file(lib/libautoware_driveworks.so ${CATKIN_DEVEL_PREFIX}/lib/libautoware_driveworks.so COPYONLY)
ELSE()
    message("autoware_driveworks_interface won't be built, CUDA and aarch64 are required.")
ENDIF ()
