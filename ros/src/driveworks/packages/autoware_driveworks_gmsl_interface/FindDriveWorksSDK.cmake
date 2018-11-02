#
# Try to find DriveWorks library
#
# Look for cmake path
<<<<<<< HEAD
find_path(DW_CMAKE_PATH 
=======
find_path(DW_CMAKE_PATH
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
 	NAMES Toolchain-V4L.cmake
	HINTS /usr/local/driveworks/samples/cmake
)
# Look for include path
<<<<<<< HEAD
find_path(DW_INCLUDE_PATH 
=======
find_path(DW_INCLUDE_PATH
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
	NAMES dw/Driveworks.h
	HINTS /usr/local/driveworks/include
)

# Look for target build system sdk
<<<<<<< HEAD
find_path(DW_MEDIA_PATH 
=======
find_path(DW_MEDIA_PATH
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
	NAMES nvmedia_image.h
	HINTS $ENV{HOME}/drive-t186ref-linux/include
	      /usr/local/drive-t186ref-linux/include
)

# Look for lib
<<<<<<< HEAD
find_path(DW_LIBRARY_PATH 
	NAMES libdriveworks.so 
=======
find_path(DW_LIBRARY_PATH
	NAMES libdriveworks.so
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
	HINTS /usr/local/driveworks/lib
)



# set cmake and include dir
IF(DW_CMAKE_PATH AND DW_INCLUDE_PATH)
        message(STATUS "FOUND CMAKE INCLUDE")
	LIST(APPEND CMAKE_MODULE_PATH ${DW_CMAKE_PATH})
	set(Driveworks_INCLUDE_DIR ${DW_INCLUDE_PATH})
        set(CMAKE_TOOLCHAIN_FILE ${DW_CMAKE_PATH}/Toolchain-V4L.cmake)
      	message(${DW_INCLUDE_PATH})
 	message(${DW_CMAKE_PATH})
ELSE(DW_CMAKE_PATH AND DW_INCLUDE_PATH)
	message(STATUS "ERROR: Check if Driveworks sdk is installed")
ENDIF(DW_CMAKE_PATH AND DW_INCLUDE_PATH)

# set nvmedia_xxx.h include dir
IF(DW_MEDIA_PATH)
    message(STATUS "FOUND TARGET")
    set(NVMEDIA_DIRS ${DW_MEDIA_PATH})
    message(${DW_MEDIA_PATH})
ELSE(DW_MEDIA_PATH)
	message(STATUS "ERROR: Check if this is drive-t186ref-linux folder")
ENDIF(DW_MEDIA_PATH)


# set driveworks lib
IF(DW_LIBRARY_PATH)
        message(STATUS "FOUND LIB")
	set(Driveworks_LIBRARY ${DW_LIBRARY_PATH})
        message(${DW_LIBRARY_PATH})
ELSE(DW_LIBRARY_PATH)
	message(STATUS "ERROR: Check if Driveworks lib is installed")
ENDIF(DW_LIBRARY_PATH)

