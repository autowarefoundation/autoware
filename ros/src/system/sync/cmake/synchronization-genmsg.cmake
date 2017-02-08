# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "synchronization: 2 messages, 0 services")

set(MSG_I_FLAGS "-Isynchronization:/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(synchronization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg" NAME_WE)
add_custom_target(_synchronization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synchronization" "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg" NAME_WE)
add_custom_target(_synchronization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "synchronization" "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synchronization
)
_generate_msg_cpp(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synchronization
)

### Generating Services

### Generating Module File
_generate_module_cpp(synchronization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synchronization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(synchronization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(synchronization_generate_messages synchronization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_cpp _synchronization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_cpp _synchronization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synchronization_gencpp)
add_dependencies(synchronization_gencpp synchronization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synchronization_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synchronization
)
_generate_msg_eus(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synchronization
)

### Generating Services

### Generating Module File
_generate_module_eus(synchronization
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synchronization
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(synchronization_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(synchronization_generate_messages synchronization_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_eus _synchronization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_eus _synchronization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synchronization_geneus)
add_dependencies(synchronization_geneus synchronization_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synchronization_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synchronization
)
_generate_msg_lisp(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synchronization
)

### Generating Services

### Generating Module File
_generate_module_lisp(synchronization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synchronization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(synchronization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(synchronization_generate_messages synchronization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_lisp _synchronization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_lisp _synchronization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synchronization_genlisp)
add_dependencies(synchronization_genlisp synchronization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synchronization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synchronization
)
_generate_msg_py(synchronization
  "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synchronization
)

### Generating Services

### Generating Module File
_generate_module_py(synchronization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synchronization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(synchronization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(synchronization_generate_messages synchronization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_monitor.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_py _synchronization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/system/sync/msg/time_diff.msg" NAME_WE)
add_dependencies(synchronization_generate_messages_py _synchronization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(synchronization_genpy)
add_dependencies(synchronization_genpy synchronization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS synchronization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synchronization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/synchronization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(synchronization_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synchronization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/synchronization
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(synchronization_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synchronization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/synchronization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(synchronization_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synchronization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synchronization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/synchronization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(synchronization_generate_messages_py std_msgs_generate_messages_py)
