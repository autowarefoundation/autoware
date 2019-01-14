cmake_minimum_required(VERSION 2.8.12)
project(pos_db)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        gnss
        jsk_recognition_msgs
        tf
        jsk_rviz_plugins
        autoware_msgs
        )

include(FindLibSsh2.cmake)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        CATKIN_DEPENDS
        roscpp
        gnss
        jsk_recognition_msgs
        tf
        autoware_msgs
)

if (LIBSSH2_FOUND)

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${LIBSSH2_INCLUDE_DIRS}
    )

    add_library(pos_db
            lib/pos_db/SendData.cpp
            lib/pos_db/util.cpp
            )
    target_link_libraries(pos_db
            ${LIBSSH2_LIBRARIES}
            )

    add_executable(pos_downloader
            nodes/pos_downloader/pos_downloader.cpp)
    target_link_libraries(pos_downloader
            pos_db
            ${catkin_LIBRARIES}
            ${LIBSSH2_LIBRARIES}
            )

    add_executable(pos_uploader
            nodes/pos_uploader/pos_uploader.cpp)
    target_link_libraries(pos_uploader
            pos_db
            ${catkin_LIBRARIES}
            ${LIBSSH2_LIBRARIES}
            )
    add_dependencies(pos_uploader
            ${catkin_EXPORTED_TARGETS}
            )

    set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

    set_target_properties(pos_uploader
            PROPERTIES COMPILE_FLAGS
            "-DCAMERA_YAML=${CAMERA_YAML}")

    install(TARGETS
            pos_downloader
            pos_db
            pos_uploader
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

    install(DIRECTORY include
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
else()
    message("LibSSH2 not found. pos_db package won't be built.")
endif()
