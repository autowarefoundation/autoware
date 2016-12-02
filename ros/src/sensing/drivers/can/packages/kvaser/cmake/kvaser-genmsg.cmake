# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kvaser: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ikvaser:/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/indigo/share/visualization_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kvaser_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg" NAME_WE)
add_custom_target(_kvaser_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kvaser" "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(kvaser
  "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kvaser
)

### Generating Services

### Generating Module File
_generate_module_cpp(kvaser
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kvaser
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kvaser_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kvaser_generate_messages kvaser_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg" NAME_WE)
add_dependencies(kvaser_generate_messages_cpp _kvaser_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kvaser_gencpp)
add_dependencies(kvaser_gencpp kvaser_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kvaser_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(kvaser
  "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kvaser
)

### Generating Services

### Generating Module File
_generate_module_eus(kvaser
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kvaser
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(kvaser_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(kvaser_generate_messages kvaser_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg" NAME_WE)
add_dependencies(kvaser_generate_messages_eus _kvaser_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kvaser_geneus)
add_dependencies(kvaser_geneus kvaser_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kvaser_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(kvaser
  "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kvaser
)

### Generating Services

### Generating Module File
_generate_module_lisp(kvaser
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kvaser
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kvaser_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kvaser_generate_messages kvaser_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg" NAME_WE)
add_dependencies(kvaser_generate_messages_lisp _kvaser_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kvaser_genlisp)
add_dependencies(kvaser_genlisp kvaser_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kvaser_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(kvaser
  "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kvaser
)

### Generating Services

### Generating Module File
_generate_module_py(kvaser
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kvaser
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kvaser_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kvaser_generate_messages kvaser_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/sensing/drivers/can/packages/kvaser/msg/CANPacket.msg" NAME_WE)
add_dependencies(kvaser_generate_messages_py _kvaser_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kvaser_genpy)
add_dependencies(kvaser_genpy kvaser_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kvaser_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kvaser)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kvaser
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(kvaser_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(kvaser_generate_messages_cpp visualization_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kvaser)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kvaser
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(kvaser_generate_messages_eus std_msgs_generate_messages_eus)
add_dependencies(kvaser_generate_messages_eus visualization_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kvaser)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kvaser
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(kvaser_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(kvaser_generate_messages_lisp visualization_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kvaser)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kvaser\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kvaser
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(kvaser_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(kvaser_generate_messages_py visualization_msgs_generate_messages_py)
