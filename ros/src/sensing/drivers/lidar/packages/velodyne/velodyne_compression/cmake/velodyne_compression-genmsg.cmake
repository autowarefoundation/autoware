# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "velodyne_compression: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ivelodyne_compression:/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(velodyne_compression_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg" NAME_WE)
add_custom_target(_velodyne_compression_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "velodyne_compression" "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(velodyne_compression
  "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_compression
)

### Generating Services

### Generating Module File
_generate_module_cpp(velodyne_compression
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_compression
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(velodyne_compression_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(velodyne_compression_generate_messages velodyne_compression_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg" NAME_WE)
add_dependencies(velodyne_compression_generate_messages_cpp _velodyne_compression_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_compression_gencpp)
add_dependencies(velodyne_compression_gencpp velodyne_compression_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_compression_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(velodyne_compression
  "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_compression
)

### Generating Services

### Generating Module File
_generate_module_eus(velodyne_compression
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_compression
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(velodyne_compression_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(velodyne_compression_generate_messages velodyne_compression_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg" NAME_WE)
add_dependencies(velodyne_compression_generate_messages_eus _velodyne_compression_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_compression_geneus)
add_dependencies(velodyne_compression_geneus velodyne_compression_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_compression_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(velodyne_compression
  "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_compression
)

### Generating Services

### Generating Module File
_generate_module_lisp(velodyne_compression
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_compression
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(velodyne_compression_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(velodyne_compression_generate_messages velodyne_compression_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg" NAME_WE)
add_dependencies(velodyne_compression_generate_messages_lisp _velodyne_compression_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_compression_genlisp)
add_dependencies(velodyne_compression_genlisp velodyne_compression_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_compression_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(velodyne_compression
  "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_compression
)

### Generating Services

### Generating Module File
_generate_module_py(velodyne_compression
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_compression
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(velodyne_compression_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(velodyne_compression_generate_messages velodyne_compression_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware_openplanner/Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_compression/msg/CompressedPacket.msg" NAME_WE)
add_dependencies(velodyne_compression_generate_messages_py _velodyne_compression_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_compression_genpy)
add_dependencies(velodyne_compression_genpy velodyne_compression_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_compression_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_compression)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_compression
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(velodyne_compression_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_compression)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_compression
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(velodyne_compression_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_compression)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_compression
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(velodyne_compression_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_compression)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_compression\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_compression
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(velodyne_compression_generate_messages_py std_msgs_generate_messages_py)
endif()
