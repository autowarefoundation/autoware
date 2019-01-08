cmake_minimum_required(VERSION 2.8.3)
project(libdpm_ttic)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        sensor_msgs
        std_msgs
        )
find_package(OpenCV REQUIRED)
find_package(CUDA)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libdpm_ttic
)

set(CMAKE_C_FLAGS "-O2 -g -Wall -Wno-unused-result ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

if(CUDA_FOUND)
    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${CUDA_INCLUDE_DIRS}
            "common"
            "cpu"
            "gpu"
    )

    if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif()

    file(GLOB DPM_TTIC_COMMON_IMPL_SOURCES "common/*.cpp")
    file(GLOB DPM_TTIC_CPU_IMPL_SOURCES "cpu/*.cpp")
    file(GLOB DPM_TTIC_GPU_IMPL_SOURCES "gpu/*.cpp")

    ## Declare a cpp library
    add_library(libdpm_ttic
            ${DPM_TTIC_COMMON_IMPL_SOURCES}
            ${DPM_TTIC_CPU_IMPL_SOURCES}
            ${DPM_TTIC_GPU_IMPL_SOURCES}
            )
    #SET_TARGET_PROPERTIES(libdpm_ttic PROPERTIES OUTPUT_NAME dpm_ttic)

    target_link_libraries(libdpm_ttic
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS}
            ${CUDA_CUDA_LIBRARY}
            )

    set(CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")

    if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        set(CUDA_LDFLAGS -L/usr/lib/arm-linux-gnueabihf/tegra -lcuda)
    else()
        set(CUDA_LDFLAGS -lcuda)
    endif()

    if(CMAKE_CROSSCOMPILING)
        if(NOT CUDA_ARCH)
                message(FATAL_ERROR "Please define the CUDA_ARCH CMake variable")
        endif()
    else()
        set(CUDA_CAPABILITY_VERSION_CHECKER
                "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker"
                )

        execute_process(
                COMMAND
                ${CUDA_COMPILER}
                -o ${CUDA_CAPABILITY_VERSION_CHECKER}
                "${CMAKE_CURRENT_SOURCE_DIR}/util/capability_version_checker.cpp"
                ${CUDA_LDFLAGS}
        )

        execute_process(
                COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
                OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
                set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
        else()
                set(CUDA_ARCH "sm_52")
        endif()
    endif()

    set(CUDA_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/gpu")
    add_custom_command(
            OUTPUT GPU_function.cubin
            COMMAND
            ${CUDA_COMPILER} "-arch=${CUDA_ARCH}"
            "-o" "${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin"
            "-cubin" "-Xptxas" "-v" "--maxrregcount" "32" "-I${CUDA_FILE_PATH}"
            "${CMAKE_CURRENT_SOURCE_DIR}/gpu/GPU_function.cu"
            DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/gpu/GPU_function.cu"
    )

    add_custom_target(cuda_binary DEPENDS GPU_function.cubin)
    add_dependencies(libdpm_ttic
            cuda_binary
            )

else()

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            "common"
            "cpu"
    )

    file(GLOB DPM_TTIC_COMMON_IMPL_SOURCES "common/*.cpp")
    file(GLOB DPM_TTIC_CPU_IMPL_SOURCES "cpu/*.cpp")

    ## Declare a cpp library
    add_library(libdpm_ttic
            ${DPM_TTIC_COMMON_IMPL_SOURCES}
            ${DPM_TTIC_CPU_IMPL_SOURCES}
            )
    #SET_TARGET_PROPERTIES(libdpm_ttic PROPERTIES OUTPUT_NAME dpm_ttic)

    target_link_libraries(libdpm_ttic
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS}
            )


endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )

install(TARGETS
        libdpm_ttic
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
