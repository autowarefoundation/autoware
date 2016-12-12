# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vehicle_socket: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ivehicle_socket:/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vehicle_socket_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg" NAME_WE)
add_custom_target(_vehicle_socket_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vehicle_socket" "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vehicle_socket
  "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vehicle_socket
)

### Generating Services

### Generating Module File
_generate_module_cpp(vehicle_socket
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vehicle_socket
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vehicle_socket_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vehicle_socket_generate_messages vehicle_socket_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg" NAME_WE)
add_dependencies(vehicle_socket_generate_messages_cpp _vehicle_socket_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vehicle_socket_gencpp)
add_dependencies(vehicle_socket_gencpp vehicle_socket_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vehicle_socket_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(vehicle_socket
  "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vehicle_socket
)

### Generating Services

### Generating Module File
_generate_module_eus(vehicle_socket
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vehicle_socket
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(vehicle_socket_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(vehicle_socket_generate_messages vehicle_socket_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg" NAME_WE)
add_dependencies(vehicle_socket_generate_messages_eus _vehicle_socket_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vehicle_socket_geneus)
add_dependencies(vehicle_socket_geneus vehicle_socket_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vehicle_socket_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vehicle_socket
  "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vehicle_socket
)

### Generating Services

### Generating Module File
_generate_module_lisp(vehicle_socket
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vehicle_socket
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vehicle_socket_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vehicle_socket_generate_messages vehicle_socket_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg" NAME_WE)
add_dependencies(vehicle_socket_generate_messages_lisp _vehicle_socket_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vehicle_socket_genlisp)
add_dependencies(vehicle_socket_genlisp vehicle_socket_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vehicle_socket_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vehicle_socket
  "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vehicle_socket
)

### Generating Services

### Generating Module File
_generate_module_py(vehicle_socket
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vehicle_socket
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vehicle_socket_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vehicle_socket_generate_messages vehicle_socket_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/socket/packages/vehicle_socket/msg/CanInfo.msg" NAME_WE)
add_dependencies(vehicle_socket_generate_messages_py _vehicle_socket_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vehicle_socket_genpy)
add_dependencies(vehicle_socket_genpy vehicle_socket_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vehicle_socket_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vehicle_socket)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vehicle_socket
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(vehicle_socket_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vehicle_socket)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vehicle_socket
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(vehicle_socket_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vehicle_socket)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vehicle_socket
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(vehicle_socket_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vehicle_socket)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vehicle_socket\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vehicle_socket
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(vehicle_socket_generate_messages_py std_msgs_generate_messages_py)
