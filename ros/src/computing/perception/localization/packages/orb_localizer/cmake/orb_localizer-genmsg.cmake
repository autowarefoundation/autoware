# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "orb_localizer: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iorb_localizer:/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(orb_localizer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg" NAME_WE)
add_custom_target(_orb_localizer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orb_localizer" "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(orb_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_localizer
)

### Generating Services

### Generating Module File
_generate_module_cpp(orb_localizer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_localizer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(orb_localizer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(orb_localizer_generate_messages orb_localizer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg" NAME_WE)
add_dependencies(orb_localizer_generate_messages_cpp _orb_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_localizer_gencpp)
add_dependencies(orb_localizer_gencpp orb_localizer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_localizer_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(orb_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_localizer
)

### Generating Services

### Generating Module File
_generate_module_eus(orb_localizer
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_localizer
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(orb_localizer_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(orb_localizer_generate_messages orb_localizer_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg" NAME_WE)
add_dependencies(orb_localizer_generate_messages_eus _orb_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_localizer_geneus)
add_dependencies(orb_localizer_geneus orb_localizer_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_localizer_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(orb_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_localizer
)

### Generating Services

### Generating Module File
_generate_module_lisp(orb_localizer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_localizer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(orb_localizer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(orb_localizer_generate_messages orb_localizer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg" NAME_WE)
add_dependencies(orb_localizer_generate_messages_lisp _orb_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_localizer_genlisp)
add_dependencies(orb_localizer_genlisp orb_localizer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_localizer_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(orb_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_localizer
)

### Generating Services

### Generating Module File
_generate_module_py(orb_localizer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_localizer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(orb_localizer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(orb_localizer_generate_messages orb_localizer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/orb_localizer/msg/debug.msg" NAME_WE)
add_dependencies(orb_localizer_generate_messages_py _orb_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_localizer_genpy)
add_dependencies(orb_localizer_genpy orb_localizer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_localizer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_localizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_localizer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(orb_localizer_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_localizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_localizer
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(orb_localizer_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_localizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_localizer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(orb_localizer_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_localizer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_localizer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_localizer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(orb_localizer_generate_messages_py std_msgs_generate_messages_py)
