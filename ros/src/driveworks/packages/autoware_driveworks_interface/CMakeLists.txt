cmake_minimum_required(VERSION 2.8.3)
project(autoware_driveworks_interface)

set(CMAKE_CXX_FLAGS "-O2 -s  -Wno-error=unused-parameter")



find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs sensor_msgs autoware_msgs
        )

find_package(CUDA)

catkin_package(
        CATKIN_DEPENDS std_msgs sensor_msgs autoware_msgs
)
find_library(DRIVEWORKS_FOUND NAMES driveworks)
if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64"
        AND CUDA_FOUND
        AND DRIVEWORKS_FOUND
)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(DRIVEWORKS_LINKER_FLAGS "-Wl,--unresolved-symbols=ignore-in-object-files -Wl,--allow-shlib-undefined")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(DRIVEWORKS_LINKER_FLAGS "-Wl,-undefined,dynamic_lookup")
    endif()


    find_package(Threads REQUIRED)
    find_package(OpenCV REQUIRED)

    find_package(catkin REQUIRED COMPONENTS
            roscpp
            cv_bridge
            std_msgs
            sensor_msgs
            tf
            )

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${autoware_msgs_INCLUDE_DIRS}
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
            ${DRIVEWORKS_LINKER_FLAGS}
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
            ${DRIVEWORKS_LINKER_FLAGS}
            )
     # Use configure_file instead of file(COPY ...) so that if the input file
     # changes, it gets copied to the DEVEL space anyway
     configure_file(lib/libautoware_driveworks.so ${CATKIN_DEVEL_PREFIX}/lib/libautoware_driveworks.so COPYONLY)

    install(TARGETS tensorrt_interface gmsl_interface
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)

    install(FILES
            lib/libautoware_driveworks.so
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib/libautoware_driveworks.so
            )
ELSE()
    message("autoware_driveworks_interface won't be built, CUDA and aarch64 are required.")
ENDIF ()
