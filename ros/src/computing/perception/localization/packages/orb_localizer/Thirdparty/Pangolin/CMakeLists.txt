cmake_minimum_required (VERSION 2.8.3)

set (LibName pangolin)

find_package (OpenGL REQUIRED)
find_package (GLEW REQUIRED)
#find_package (GLUT REQUIRED)


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

set (SRCDIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

embed_binary_files( "${SRCDIR}/_embed_/fonts/*.ttf" "${CMAKE_CURRENT_BINARY_DIR}/fonts.c" )
list ( APPEND SOURCES "${CMAKE_CURRENT_BINARY_DIR}/fonts.c" )

append_glob (SOURCES ${SRCDIR}/utils/*.cpp)
append_glob (SOURCES ${SRCDIR}/image/*.cpp)
append_glob (SOURCES ${SRCDIR}/log/*.cpp)
append_glob (SOURCES ${SRCDIR}/gl/*.cpp)
append_glob (SOURCES ${SRCDIR}/display/*.cpp)
append_glob (SOURCES ${SRCDIR}/handler/*.cpp)
append_glob (SOURCES ${SRCDIR}/var/*.cpp)
append_glob (SOURCES ${SRCDIR}/display/widgets/*.cpp)
list (APPEND SOURCES ${SRCDIR}/display/device/display_x11.cpp)

#set (HAVE_GLUT 1)
set (HAVE_GLEW 1)
#message (STATUS "GLUT Enabled")

configure_file (
	src/config.h.in
	${CMAKE_CURRENT_SOURCE_DIR}/include/pangolin/config.h
)


include_directories (
	include
)

add_library (${LibName} STATIC ${SOURCES})
set_target_properties(${LibName} PROPERTIES LINKER_LANGUAGE CXX)
