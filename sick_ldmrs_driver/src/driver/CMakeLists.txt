cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(LIBSICK_LDMRS)

set(SICK_LDMRS_MAJOR_VERSION 0)
set(SICK_LDMRS_MINOR_VERSION 1)
set(SICK_LDMRS_PATCH_VERSION 0)
set(SICK_LDMRS_VERSION
  ${SICK_LDMRS_MAJOR_VERSION}.${SICK_LDMRS_MINOR_VERSION}.${SICK_LDMRS_PATCH_VERSION})

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/SickLDMRS)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# set up include-directories
include_directories("${PROJECT_SOURCE_DIR}/include")

# Extra cmake macros
INCLUDE (CheckSymbolExists)

# Add sub-directories
add_subdirectory(src)

###### create and install Config module ####

# Add all targets to the build-tree export set
export(TARGETS sick_ldmrs LDMRS_Example
  FILE "${PROJECT_BINARY_DIR}/SickLDMRSTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE SickLDMRS)

# Create the SickLDMRSConfig.cmake and SickLDMRSConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(SickLDMRSConfig.cmake.in
  "${PROJECT_BINARY_DIR}/SickLDMRSConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${SICK_LDMRS_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(SickLDMRSConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/SickLDMRSConfig.cmake" @ONLY)
# ... for both
configure_file(SickLDMRSConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/SickLDMRSConfigVersion.cmake" @ONLY)

# Install the SickLDMRSConfig.cmake and SickLDMRSConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/SickLDMRSConfig.cmake"
  "${PROJECT_BINARY_DIR}/SickLDMRSConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT SickLDMRSTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install catkin package.xml
install(FILES package.xml DESTINATION share/sick_ldmrs)
