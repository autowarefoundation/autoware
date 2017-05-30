# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "icp_localizer: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iicp_localizer:/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(icp_localizer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg" NAME_WE)
add_custom_target(_icp_localizer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "icp_localizer" "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(icp_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/icp_localizer
)

### Generating Services

### Generating Module File
_generate_module_cpp(icp_localizer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/icp_localizer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(icp_localizer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(icp_localizer_generate_messages icp_localizer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg" NAME_WE)
add_dependencies(icp_localizer_generate_messages_cpp _icp_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(icp_localizer_gencpp)
add_dependencies(icp_localizer_gencpp icp_localizer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS icp_localizer_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(icp_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/icp_localizer
)

### Generating Services

### Generating Module File
_generate_module_eus(icp_localizer
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/icp_localizer
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(icp_localizer_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(icp_localizer_generate_messages icp_localizer_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg" NAME_WE)
add_dependencies(icp_localizer_generate_messages_eus _icp_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(icp_localizer_geneus)
add_dependencies(icp_localizer_geneus icp_localizer_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS icp_localizer_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(icp_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/icp_localizer
)

### Generating Services

### Generating Module File
_generate_module_lisp(icp_localizer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/icp_localizer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(icp_localizer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(icp_localizer_generate_messages icp_localizer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg" NAME_WE)
add_dependencies(icp_localizer_generate_messages_lisp _icp_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(icp_localizer_genlisp)
add_dependencies(icp_localizer_genlisp icp_localizer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS icp_localizer_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(icp_localizer
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/icp_localizer
)

### Generating Services

### Generating Module File
_generate_module_py(icp_localizer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/icp_localizer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(icp_localizer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(icp_localizer_generate_messages icp_localizer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/localization/packages/icp_localizer/msg/icp_stat.msg" NAME_WE)
add_dependencies(icp_localizer_generate_messages_py _icp_localizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(icp_localizer_genpy)
add_dependencies(icp_localizer_genpy icp_localizer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS icp_localizer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/icp_localizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/icp_localizer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(icp_localizer_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/icp_localizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/icp_localizer
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(icp_localizer_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/icp_localizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/icp_localizer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(icp_localizer_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/icp_localizer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/icp_localizer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/icp_localizer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(icp_localizer_generate_messages_py std_msgs_generate_messages_py)
endif()
