cmake_minimum_required(VERSION 2.8.3)
project(ndt_gpu)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags)
find_package(PCL REQUIRED)
find_package(CUDA)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

AW_CHECK_CUDA()

if (USE_CUDA)
    set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

    if(CMAKE_CROSSCOMPILING)
        if(NOT CUDA_ARCH)
                message(FATAL_ERROR "Please define the CUDA_ARCH CMake variable")
        endif()
    else()
        if (NOT DEFINED CUDA_CAPABILITY_VERSION_CHECKER)
            set(CUDA_CAPABILITY_VERSION_CHECKER
                    "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker")
        endif ()

        execute_process(COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
                OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE)

        if ("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
            set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
        else ()
            set(CUDA_ARCH "sm_52")
        endif ()
    endif()

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11;--ptxas-options=-v)

    set(SUBSYS_DESC "Point cloud ndt gpu library")

    catkin_package(
            DEPENDS PCL                                #Non-catkin CMake projects
            INCLUDE_DIRS include                        #The exported include paths
            LIBRARIES ndt_gpu                           #The exported libraries from the project
    )

    include_directories(
            ${PCL_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
            ${CUDA_INCLUDE_DIRS}
            include
            ${EIGEN3_INCLUDE_DIRS}
    )

    set(srcs
            src/MatrixDevice.cu
            src/MatrixHost.cu
            src/NormalDistributionsTransform.cu
            src/Registration.cu
            src/VoxelGrid.cu
            src/SymmetricEigenSolver.cu
            )

    set(incs
            include/ndt_gpu/common.h
            include/ndt_gpu/debug.h
            include/ndt_gpu/Matrix.h
            include/ndt_gpu/MatrixDevice.h
            include/ndt_gpu/MatrixHost.h
            include/ndt_gpu/NormalDistributionsTransform.h
            include/ndt_gpu/Registration.h
            include/ndt_gpu/SymmetricEigenSolver.h
            include/ndt_gpu/VoxelGrid.h
            )

    cuda_add_library(ndt_gpu ${srcs} ${incs})

    target_link_libraries(ndt_gpu
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${PCL_LIBRARIES}
            )

    install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            FILES_MATCHING PATTERN "*.h"
            )


    install(TARGETS ndt_gpu
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

else ()
    message("ndt_gpu will not be built, CUDA was not found.")
endif ()