SET (_UNIX_ 1)
SET (_GCC_ 1)
SET (BUILD_PANGOLIN_GUI 1)
SET (BUILD_PANGOLIN_VARS 1)

macro( append_glob list glob )
    file(GLOB files ${glob})
    set(${list} "${${list}};${files}")
endmacro()


# Creates C resources file from files in given directory
# Based on http://stackoverflow.com/a/27206982
function(embed_binary_files file_glob output)
    # Create empty output file
    file(WRITE ${output} "")
    # Collect input files
    file(GLOB bins ${file_glob})
    # Iterate through input files
    foreach(bin ${bins})
        # Get short filename
        string(REGEX MATCH "([^/]+)$" filename ${bin})
        # Replace filename spaces & extension separator for C compatibility
        string(REGEX REPLACE "\\.| " "_" filename ${filename})
        # Read hex data from file
        file(READ ${bin} filedata HEX)
        # Convert hex data for C compatibility
        string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," filedata ${filedata})
        # Append data to output file
        file(APPEND ${output} "const unsigned char ${filename}[] = {${filedata}};\nconst unsigned ${filename}_size = sizeof(${filename});\n")
    endforeach()
endfunction()

## Header only includes / core headers
set( INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Pangolin/include/pangolin )
set( HEADERS
  ${INCDIR}/pangolin.h
  ${INCDIR}/platform.h
)
set ( SRCDIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Pangolin/src )

append_glob(HEADERS ${INCDIR}/utils/*.h*)
append_glob(HEADERS ${INCDIR}/image/*.h*)
append_glob(HEADERS ${INCDIR}/log/*.h*)

### Store list of source files
append_glob(SOURCES ${SRCDIR}/utils/*.cpp)
append_glob(SOURCES ${SRCDIR}/image/*.cpp)
append_glob(SOURCES ${SRCDIR}/log/*.cpp)

# GUI Parts
append_glob(HEADERS ${INCDIR}/gl/*.h*)
append_glob(HEADERS ${INCDIR}/display/*.h*)
append_glob(HEADERS ${INCDIR}/handler/*.h*)

embed_binary_files( "${SRCDIR}/_embed_/fonts/*.ttf" "${CMAKE_CURRENT_BINARY_DIR}/fonts.c" )
list(APPEND SOURCES "${CMAKE_CURRENT_BINARY_DIR}/fonts.c" )

append_glob(SOURCES ${SRCDIR}/gl/*.cpp)
append_glob(SOURCES ${SRCDIR}/display/*.cpp)
append_glob(SOURCES ${SRCDIR}/handler/*.cpp)

append_glob(HEADERS ${INCDIR}/var/*.h*)
append_glob(SOURCES ${SRCDIR}/var/*.cpp)

list(APPEND HEADERS ${INCDIR}/display/widgets/widgets.h )
append_glob(SOURCES ${SRCDIR}/display/widgets/*.cpp )

find_package (OpenGL REQUIRED)
find_package (GLEW REQUIRED)
list(APPEND USER_INC  "${GLEW_INCLUDE_DIR}" )
list(APPEND LINK_LIBS "${GLEW_LIBRARY}" )
find_package (FREEGLUT REQUIRED)
if(FREEGLUT_FOUND)
	set(HAVE_GLUT 1)
	list(APPEND HEADERS ${INCDIR}/display/device/display_glut.h )
	list(APPEND SOURCES ${SRCDIR}/display/device/display_glut.cpp )
	message(STATUS "Glut Found and Enabled")
endif()
set(HAVE_GLEW 1)

set (LibName pangolin)

configure_file(
  "${SRCDIR}/config.h.in"
  "${INCDIR}/config.h"
)

add_library (${LibName} STATIC ${SOURCES})
set_target_properties(${LibName} PROPERTIES LINKER_LANGUAGE CXX)
