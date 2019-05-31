cmake_minimum_required(VERSION 2.8.3)
project(autoware_driveworks_gmsl_interface)

find_package(CUDA)
find_library(DRIVEWORKS_FOUND NAMES driveworks)
find_library(NVMEDIA_FOUND NAMES nvmedia)
if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64"
        AND CUDA_FOUND
        AND DRIVEWORKS_FOUND
        AND NVMEDIA_FOUND
)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(DRIVEWORKS_LINKER_FLAGS "-Wl,--unresolved-symbols=ignore-in-object-files -Wl,--allow-shlib-undefined")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(DRIVEWORKS_LINKER_FLAGS "-Wl,-undefined,dynamic_lookup")
    endif()

    find_package(catkin REQUIRED COMPONENTS
            autoware_build_flags
            roscpp
            sensor_msgs
            std_msgs
            cv_bridge
            image_transport
            camera_info_manager
            nodelet
    )

    catkin_package(
            LIBRARIES ${PROJECT_NAME}
            CATKIN_DEPENDS
            sensor_msgs
            std_msgs
    )

    add_definitions(-DVIBRANTE)

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${DRIVEWORKS_TOOLKIT_TARGET_DIR}/include
        ${DRIVE_T186REF_LINUX_ROOT_DIR}/include
    )

    link_directories(
        ${catkin_LIBRARY_DIRS}
        /usr/lib/
        lib
        /usr/local/driveworks/lib/
        ${DRIVEWORKS_TOOLKIT_TARGET_DIR}/lib
        ${DRIVE_T186REF_LINUX_ROOT_DIR}/lib-target
    )

    set(NODE_NAME ${PROJECT_NAME}_node)
    cuda_add_executable(${NODE_NAME}
        src/gmsl_camera_multiple/gmsl_node.cpp
        src/gmsl_camera_multiple/cv_connection.cpp
        src/gmsl_camera_multiple/SekonixGmslCamera.cpp
        src/gmsl_camera_multiple/DriveWorksApi.cpp
        src/gmsl_camera_multiple/DeviceArguments.cpp
    )

    add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${NODE_NAME} PRIVATE
        ${catkin_LIBRARIES}
        nvmedia
        driveworks
        ${DRIVEWORKS_LINKER_FLAGS}
    )

    set(NODELET_NAME nodelet_${PROJECT_NAME})
    add_library(${NODELET_NAME}
        src/gmsl_camera_multiple/gmsl_nodelets.cpp
        src/gmsl_camera_multiple/cv_connection.cpp
        src/gmsl_camera_multiple/SekonixGmslCamera.cpp
        src/gmsl_camera_multiple/DriveWorksApi.cpp
        src/gmsl_camera_multiple/DeviceArguments.cpp
    )

    target_link_libraries(${NODELET_NAME}
        ${catkin_LIBRARIES}
        nvmedia
        driveworks
        ${DRIVEWORKS_LINKER_FLAGS}
    )

    install(TARGETS ${NODE_NAME} ${NODELET_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)

else()
    find_package(catkin REQUIRED)
    catkin_package()
    message("gmsl_camera driver will not be built. CUDA was not found.")
endif()

install(FILES
        nodelet_gmsl_camera.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )