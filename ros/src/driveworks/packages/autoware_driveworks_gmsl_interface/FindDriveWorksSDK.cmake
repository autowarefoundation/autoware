#
# Try to find DriveWorks sdk
#


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



# set include dir
IF(DW_INCLUDE_PATH)
	message(STATUS "FOUND CMAKE INCLUDE")
	set(Driveworks_INCLUDE_DIR ${DW_INCLUDE_PATH})
    message(${DW_INCLUDE_PATH})
ELSE(DW_INCLUDE_PATH)
	message(STATUS "ERROR: Check if Driveworks sdk is installed")
ENDIF(DW_INCLUDE_PATH)

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
	set(Driveworks_LIBRARIES ${DW_LIBRARY_PATH})
    message(${DW_LIBRARY_PATH})
ELSE(DW_LIBRARY_PATH)
	message(STATUS "ERROR: Check if Driveworks lib is installed")
ENDIF(DW_LIBRARY_PATH)

