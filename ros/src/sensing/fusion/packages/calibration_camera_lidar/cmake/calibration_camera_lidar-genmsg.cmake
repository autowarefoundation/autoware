# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "calibration_camera_lidar: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icalibration_camera_lidar:/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(calibration_camera_lidar_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg" NAME_WE)
add_custom_target(_calibration_camera_lidar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "calibration_camera_lidar" "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(calibration_camera_lidar
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration_camera_lidar
)

### Generating Services

### Generating Module File
_generate_module_cpp(calibration_camera_lidar
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration_camera_lidar
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(calibration_camera_lidar_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(calibration_camera_lidar_generate_messages calibration_camera_lidar_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg" NAME_WE)
add_dependencies(calibration_camera_lidar_generate_messages_cpp _calibration_camera_lidar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_camera_lidar_gencpp)
add_dependencies(calibration_camera_lidar_gencpp calibration_camera_lidar_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_camera_lidar_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(calibration_camera_lidar
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration_camera_lidar
)

### Generating Services

### Generating Module File
_generate_module_eus(calibration_camera_lidar
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration_camera_lidar
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(calibration_camera_lidar_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(calibration_camera_lidar_generate_messages calibration_camera_lidar_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg" NAME_WE)
add_dependencies(calibration_camera_lidar_generate_messages_eus _calibration_camera_lidar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_camera_lidar_geneus)
add_dependencies(calibration_camera_lidar_geneus calibration_camera_lidar_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_camera_lidar_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(calibration_camera_lidar
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration_camera_lidar
)

### Generating Services

### Generating Module File
_generate_module_lisp(calibration_camera_lidar
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration_camera_lidar
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(calibration_camera_lidar_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(calibration_camera_lidar_generate_messages calibration_camera_lidar_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg" NAME_WE)
add_dependencies(calibration_camera_lidar_generate_messages_lisp _calibration_camera_lidar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_camera_lidar_genlisp)
add_dependencies(calibration_camera_lidar_genlisp calibration_camera_lidar_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_camera_lidar_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(calibration_camera_lidar
  "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration_camera_lidar
)

### Generating Services

### Generating Module File
_generate_module_py(calibration_camera_lidar
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration_camera_lidar
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(calibration_camera_lidar_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(calibration_camera_lidar_generate_messages calibration_camera_lidar_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/sensing/fusion/packages/calibration_camera_lidar/msg/projection_matrix.msg" NAME_WE)
add_dependencies(calibration_camera_lidar_generate_messages_py _calibration_camera_lidar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(calibration_camera_lidar_genpy)
add_dependencies(calibration_camera_lidar_genpy calibration_camera_lidar_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS calibration_camera_lidar_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration_camera_lidar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/calibration_camera_lidar
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(calibration_camera_lidar_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration_camera_lidar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/calibration_camera_lidar
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(calibration_camera_lidar_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration_camera_lidar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/calibration_camera_lidar
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(calibration_camera_lidar_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration_camera_lidar)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration_camera_lidar\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/calibration_camera_lidar
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(calibration_camera_lidar_generate_messages_py std_msgs_generate_messages_py)
