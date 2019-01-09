cmake_minimum_required(VERSION 2.8.3)
project(requirements_version_checker)

find_package(OpenCV REQUIRED)

set(ROS_VERSION $ENV{ROS_DISTRO})

if (NOT "${ROS_VERSION}" MATCHES "(kinetic)")
    message(
            FATAL_ERROR
            "You must need ROS kinetic (Your ROS version is '${ROS_VERSION}')"
    )
endif ()
