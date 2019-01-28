cmake_minimum_required(VERSION 2.8.12)
project(vision_dpm_ttic_detect)

include(FindPkgConfig)

find_package(autoware_msgs REQUIRED)

find_package(autoware_config_msgs REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        sensor_msgs
        std_msgs
        libdpm_ttic
        )

FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS cv_bridge
        roscpp
        sensor_msgs
        std_msgs
        autoware_config_msgs
        autoware_msgs
        libdpm_ttic
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

IF (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()
ENDIF ()

## dpm_ttic ##
#this must be fixed to use cmake cuda commands
IF (CUDA_FOUND)

    cuda_add_executable(vision_dpm_ttic_detect
            nodes/vision_dpm_ttic_detect/vision_dpm_ttic_detect.cpp
            )

    target_link_libraries(vision_dpm_ttic_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            )

    set_target_properties(vision_dpm_ttic_detect
            PROPERTIES
            COMPILE_FLAGS
            "-DHAS_GPU -DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
            )
ELSE ()

    add_executable(vision_dpm_ttic_detect
            nodes/vision_dpm_ttic_detect/vision_dpm_ttic_detect.cpp
            )

    target_link_libraries(vision_dpm_ttic_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            )

    set_target_properties(vision_dpm_ttic_detect
            PROPERTIES
            COMPILE_FLAGS
            "-DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
            )
ENDIF ()

add_dependencies(vision_dpm_ttic_detect
        ${catkin_EXPORTED_TARGETS}
        autoware_msgs_generate_messages_cpp
        )

install(TARGETS vision_dpm_ttic_detect
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
