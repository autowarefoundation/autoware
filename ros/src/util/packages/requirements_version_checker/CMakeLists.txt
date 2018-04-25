cmake_minimum_required(VERSION 2.8.3)
project(requirements_version_checker)

find_package(OpenCV REQUIRED)

execute_process(
        COMMAND rosversion -d
        OUTPUT_VARIABLE ROS_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (NOT "${ROS_VERSION}" MATCHES "(indigo|jade|kinetic)")
    message(
            FATAL_ERROR
            "You must need ROS indigo or jade or kinetic(Your ROS version is '${ROS_VERSION}')"
    )
endif ()