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

option(CPU_ONLY "CPU_Only" TRUE)

####### Melodic Checks
if (NOT CPU_ONLY AND CUDA_FOUND AND "$ENV{ROS_DISTRO}" STREQUAL "melodic" )
  if(${CMAKE_VERSION} VERSION_LESS "3.12.3")
    message(FATAL_ERROR "GPU support on Melodic requires CMake version>= 3.12.3")
  else()
    if (${EIGEN3_VERSION_STRING} VERSION_LESS "3.3.6")
      message(FATAL_ERROR "GPU support on Melodic requires Eigen version>= 3.3.6")
    endif()
  endif()
else()
  message(WARNING "Building on CPU only mode. Use -DCPU_ONLY=OFF to enable GPU support.")
endif()
####### End Melodic Checks