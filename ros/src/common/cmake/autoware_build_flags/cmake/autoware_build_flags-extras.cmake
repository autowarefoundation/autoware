if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
endif()

# Enable support for C++11
if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(CUDA QUIET)
find_package(Eigen3 QUIET)

option(ENABLE_CUDA "Enable_CUDA" FALSE)

####### Melodic Checks
if (ENABLE_CUDA AND CUDA_FOUND AND "$ENV{ROS_DISTRO}" STREQUAL "melodic" )
  if(${CUDA_VERSION} VERSION_GREATER "9.1" AND ${CMAKE_VERSION} VERSION_LESS "3.12.3")
    unset(CUDA_cublas_device_LIBRARY CACHE)
    set(CUDA_cublas_device_LIBRARY ${CUDA_cublas_LIBRARY})
    set(CUDA_CUBLAS_LIBRARIES ${CUDA_cublas_LIBRARY})
  endif()
  if (${EIGEN3_VERSION_STRING} VERSION_LESS "3.3.6")
      message(FATAL_ERROR "GPU support on Melodic requires Eigen version>= 3.3.6")
  endif()
else()
  message(WARNING "CUDA support is disabled. Use -DENABLE_CUDA=TRUE and install Eigen >= 3.3.6 to enable it")
endif()
####### End Melodic Checks