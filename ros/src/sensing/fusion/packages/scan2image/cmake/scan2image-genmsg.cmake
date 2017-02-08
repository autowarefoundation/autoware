# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "scan2image: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iscan2image:/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(scan2image_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg" NAME_WE)
add_custom_target(_scan2image_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "scan2image" "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(scan2image
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scan2image
)

### Generating Services

### Generating Module File
_generate_module_cpp(scan2image
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scan2image
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(scan2image_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(scan2image_generate_messages scan2image_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg" NAME_WE)
add_dependencies(scan2image_generate_messages_cpp _scan2image_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scan2image_gencpp)
add_dependencies(scan2image_gencpp scan2image_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scan2image_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(scan2image
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scan2image
)

### Generating Services

### Generating Module File
_generate_module_eus(scan2image
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scan2image
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(scan2image_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(scan2image_generate_messages scan2image_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg" NAME_WE)
add_dependencies(scan2image_generate_messages_eus _scan2image_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scan2image_geneus)
add_dependencies(scan2image_geneus scan2image_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scan2image_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(scan2image
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scan2image
)

### Generating Services

### Generating Module File
_generate_module_lisp(scan2image
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scan2image
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(scan2image_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(scan2image_generate_messages scan2image_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg" NAME_WE)
add_dependencies(scan2image_generate_messages_lisp _scan2image_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scan2image_genlisp)
add_dependencies(scan2image_genlisp scan2image_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scan2image_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(scan2image
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scan2image
)

### Generating Services

### Generating Module File
_generate_module_py(scan2image
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scan2image
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(scan2image_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(scan2image_generate_messages scan2image_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/scan2image/msg/ScanImage.msg" NAME_WE)
add_dependencies(scan2image_generate_messages_py _scan2image_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scan2image_genpy)
add_dependencies(scan2image_genpy scan2image_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scan2image_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scan2image)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scan2image
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(scan2image_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(scan2image_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scan2image)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scan2image
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(scan2image_generate_messages_eus std_msgs_generate_messages_eus)
add_dependencies(scan2image_generate_messages_eus sensor_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scan2image)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scan2image
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(scan2image_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(scan2image_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scan2image)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scan2image\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scan2image
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(scan2image_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(scan2image_generate_messages_py sensor_msgs_generate_messages_py)
