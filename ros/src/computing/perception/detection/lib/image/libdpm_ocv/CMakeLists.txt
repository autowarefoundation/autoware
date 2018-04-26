cmake_minimum_required(VERSION 2.8.3)
project(libdpm_ocv)

set(ROS_VERSION $ENV{ROS_DISTRO})

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        autoware_msgs
        )

find_package(CUDA)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libdpm_ocv
        CATKIN_DEPENDS std_msgs autoware_msgs
)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")

    SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -std=c++11 -DUSE_GPU -DCUBIN_PATH=${CATKIN_DEVEL_PREFIX}/lib/gpu_function.cubin ${CMAKE_CXX_FLAGS}")
    SET(CMAKE_NVCC_FLAGS "-use_fast_math")

    IF (CUDA_FOUND)
        INCLUDE_DIRECTORIES(
                ${catkin_INCLUDE_DIRS}
                include
                ${CUDA_INCLUDE_DIRS}
        )

        ADD_LIBRARY(libdpm_ocv
                gpu/gpu_init.cpp
                gpu/dpm_ocv_gpu.cpp
                gpu/featurepyramid_gpu.cpp
                gpu/matching_gpu.cpp
                gpu/resizeimg_gpu.cpp
                gpu/routine_gpu.cpp
                cpu/dpm_ocv_cpu.cpp
                )

        SET_TARGET_PROPERTIES(libdpm_ocv
                PROPERTIES COMPILE_FLAGS
                "-DDPM_GPU_ROOT=${DPM_GPU_ROOT}"
                )

        TARGET_LINK_LIBRARIES(libdpm_ocv
                ${catkin_LIBRARIES}
                ${OpenCV_LIBS}
                cuda
                )

        SET(CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")

        IF ("${ARCHITECTURE}" MATCHES "^arm")
            SET(CUDA_LDFLAGS -L/usr/lib/arm-linux-gnueabihf/tegra -lcuda)
        ELSE ()
            SET(CUDA_LDFLAGS -lcuda)
        ENDIF ()

        SET(CUDA_CAPABILITY_VERSION_CHECKER
                "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker"
                )

        EXECUTE_PROCESS(
                COMMAND
                ${CUDA_COMPILER}
                -o ${CUDA_CAPABILITY_VERSION_CHECKER}
                "${CMAKE_CURRENT_SOURCE_DIR}/../dpm_ttic/util/capability_version_checker.cpp"
                ${CUDA_LDFLAGS}
        )

        EXECUTE_PROCESS(
                COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
                OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        IF ("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
            SET(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
        ELSE ()
            SET(CUDA_ARCH "sm_52")
        ENDIF ()

        ADD_CUSTOM_COMMAND(
                OUTPUT gpu_function.cubin
                COMMAND
                ${CUDA_COMPILER} "-arch=${CUDA_ARCH}"
                "-o" "${CATKIN_DEVEL_PREFIX}/lib/gpu_function.cubin"
                "-cubin" "-Xptxas" "-v" "--maxrregcount" "32"
                "${CMAKE_CURRENT_SOURCE_DIR}/gpu/gpu_function.cu" "${CMAKE_NVCC_FLAGS}"
                DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/gpu/gpu_function.cu"
        )

        ADD_CUSTOM_TARGET(cuda_binary_ DEPENDS gpu_function.cubin)
        ADD_DEPENDENCIES(libdpm_ocv cuda_binary_)

    ELSE ()#NO CUDA

        INCLUDE_DIRECTORIES(
                ${catkin_INCLUDE_DIRS}
                include
        )

        ADD_LIBRARY(libdpm_ocv
                cpu/dpm_ocv_cpu.cpp
                )

        TARGET_LINK_LIBRARIES(libdpm_ocv
                ${catkin_LIBRARIES}
                )

    ENDIF ()

    ADD_DEPENDENCIES(libdpm_ocv ${catkin_EXPORTED_TARGETS})

    install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            FILES_MATCHING PATTERN "*.hpp"
            )

    install(TARGETS
            libdpm_ocv
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
            )

endif ()
