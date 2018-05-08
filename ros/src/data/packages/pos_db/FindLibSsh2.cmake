# - Try to find libssh2
# Once done this will define
#  LIBSSH2_FOUND - System has libssh2
#  LIBSSH2_INCLUDE_DIRS - The libssh2 include directories
#  LIBSSH2_LIBRARIES - The libraries needed to use libssh2
#  LIBSSH2_DEFINITIONS - Compiler switches required for using libssh2

find_package(PkgConfig)
pkg_check_modules(PC_LIBSSH2 QUIET libssh2)
set(LIBSSH2_DEFINITIONS ${PC_LIBSSH2_CFLAGS_OTHER})

message(STATUS "Looking for libssh2...")
find_path(LIBSSH2_INCLUDE_DIR libssh2.h
        HINTS ${PC_LIBSSH2_INCLUDEDIR} ${PC_LIBSSH2_INCLUDE_DIRS}
        PATH_SUFFIXES libssh2 )

find_library(LIBSSH2_LIBRARY NAMES ssh2 libssh2
        HINTS ${PC_LIBSSH2_LIBDIR} ${PC_LIBSSH2_LIBRARY_DIRS} )

set(LIBSSH2_LIBRARIES ${LIBSSH2_LIBRARY} )
set(LIBSSH2_INCLUDE_DIRS ${LIBSSH2_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBSSH2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibSsh2  DEFAULT_MSG
        LIBSSH2_LIBRARY LIBSSH2_INCLUDE_DIR)

mark_as_advanced(LIBSSH2_INCLUDE_DIR LIBSSH2_LIBRARY)

