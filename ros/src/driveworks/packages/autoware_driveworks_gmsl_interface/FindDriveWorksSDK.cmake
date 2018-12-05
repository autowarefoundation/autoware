#
# Try to find DriveWorks library
#
# Look for cmake path
find_path(DW_CMAKE_PATH
 	NAMES Toolchain-V4L.cmake
	HINTS /usr/local/driveworks/samples/cmake
)
# Look for include path
find_path(DW_INCLUDE_PATH
	NAMES dw/Driveworks.h
	HINTS /usr/local/driveworks/include
)

# Look for target build system sdk
find_path(DW_MEDIA_PATH
	NAMES nvmedia_image.h
	HINTS $ENV{HOME}/drive-t186ref-linux/include
	      /usr/local/drive-t186ref-linux/include
)

# Look for lib
find_path(DW_LIBRARY_PATH
	NAMES libdriveworks.so
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

