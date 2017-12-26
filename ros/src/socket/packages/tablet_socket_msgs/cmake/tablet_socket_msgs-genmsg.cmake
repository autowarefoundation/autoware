# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tablet_socket_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Itablet_socket_msgs:/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tablet_socket_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg" NAME_WE)
add_custom_target(_tablet_socket_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tablet_socket_msgs" "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg" NAME_WE)
add_custom_target(_tablet_socket_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tablet_socket_msgs" "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg" NAME_WE)
add_custom_target(_tablet_socket_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tablet_socket_msgs" "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg" "tablet_socket_msgs/Waypoint:std_msgs/Header"
)

get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg" NAME_WE)
add_custom_target(_tablet_socket_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tablet_socket_msgs" "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg" NAME_WE)
add_custom_target(_tablet_socket_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tablet_socket_msgs" "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg" ""
)

get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg" NAME_WE)
add_custom_target(_tablet_socket_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tablet_socket_msgs" "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_cpp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_cpp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_cpp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_cpp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_cpp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(tablet_socket_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tablet_socket_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tablet_socket_msgs_generate_messages tablet_socket_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_cpp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_cpp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_cpp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_cpp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_cpp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_cpp _tablet_socket_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tablet_socket_msgs_gencpp)
add_dependencies(tablet_socket_msgs_gencpp tablet_socket_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tablet_socket_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_eus(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_eus(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_eus(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_eus(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_eus(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(tablet_socket_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tablet_socket_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tablet_socket_msgs_generate_messages tablet_socket_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_eus _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_eus _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_eus _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_eus _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_eus _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_eus _tablet_socket_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tablet_socket_msgs_geneus)
add_dependencies(tablet_socket_msgs_geneus tablet_socket_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tablet_socket_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_lisp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_lisp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_lisp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_lisp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_lisp(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(tablet_socket_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tablet_socket_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tablet_socket_msgs_generate_messages tablet_socket_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_lisp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_lisp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_lisp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_lisp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_lisp _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_lisp _tablet_socket_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tablet_socket_msgs_genlisp)
add_dependencies(tablet_socket_msgs_genlisp tablet_socket_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tablet_socket_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_py(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_py(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_py(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_py(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
)
_generate_msg_py(tablet_socket_msgs
  "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(tablet_socket_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tablet_socket_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tablet_socket_msgs_generate_messages tablet_socket_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_py _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/gear_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_py _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/route_cmd.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_py _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/error_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_py _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_py _tablet_socket_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/socket/packages/tablet_socket_msgs/msg/mode_info.msg" NAME_WE)
add_dependencies(tablet_socket_msgs_generate_messages_py _tablet_socket_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tablet_socket_msgs_genpy)
add_dependencies(tablet_socket_msgs_genpy tablet_socket_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tablet_socket_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tablet_socket_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tablet_socket_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tablet_socket_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tablet_socket_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tablet_socket_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tablet_socket_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tablet_socket_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tablet_socket_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
