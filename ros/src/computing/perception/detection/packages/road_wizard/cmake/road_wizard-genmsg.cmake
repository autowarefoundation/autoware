# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "road_wizard: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iroad_wizard:/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(road_wizard_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg" NAME_WE)
add_custom_target(_road_wizard_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "road_wizard" "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg" ""
)

get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg" NAME_WE)
add_custom_target(_road_wizard_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "road_wizard" "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg" "road_wizard/ValueSet:road_wizard/ColorSet:std_msgs/Header"
)

get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg" NAME_WE)
add_custom_target(_road_wizard_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "road_wizard" "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg" "std_msgs/Header:road_wizard/ExtractedPosition"
)

get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg" NAME_WE)
add_custom_target(_road_wizard_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "road_wizard" "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg" "road_wizard/ValueSet"
)

get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg" NAME_WE)
add_custom_target(_road_wizard_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "road_wizard" "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard
)
_generate_msg_cpp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard
)
_generate_msg_cpp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard
)
_generate_msg_cpp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard
)
_generate_msg_cpp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard
)

### Generating Services

### Generating Module File
_generate_module_cpp(road_wizard
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(road_wizard_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(road_wizard_generate_messages road_wizard_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_cpp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_cpp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_cpp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_cpp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_cpp _road_wizard_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(road_wizard_gencpp)
add_dependencies(road_wizard_gencpp road_wizard_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS road_wizard_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard
)
_generate_msg_eus(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard
)
_generate_msg_eus(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard
)
_generate_msg_eus(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard
)
_generate_msg_eus(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard
)

### Generating Services

### Generating Module File
_generate_module_eus(road_wizard
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(road_wizard_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(road_wizard_generate_messages road_wizard_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_eus _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_eus _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_eus _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_eus _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_eus _road_wizard_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(road_wizard_geneus)
add_dependencies(road_wizard_geneus road_wizard_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS road_wizard_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard
)
_generate_msg_lisp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard
)
_generate_msg_lisp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard
)
_generate_msg_lisp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard
)
_generate_msg_lisp(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard
)

### Generating Services

### Generating Module File
_generate_module_lisp(road_wizard
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(road_wizard_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(road_wizard_generate_messages road_wizard_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_lisp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_lisp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_lisp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_lisp _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_lisp _road_wizard_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(road_wizard_genlisp)
add_dependencies(road_wizard_genlisp road_wizard_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS road_wizard_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard
)
_generate_msg_py(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard
)
_generate_msg_py(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard
)
_generate_msg_py(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg"
  "${MSG_I_FLAGS}"
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard
)
_generate_msg_py(road_wizard
  "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard
)

### Generating Services

### Generating Module File
_generate_module_py(road_wizard
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(road_wizard_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(road_wizard_generate_messages road_wizard_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ExtractedPosition.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_py _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/TunedResult.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_py _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/Signals.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_py _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ColorSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_py _road_wizard_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hatem/autoware-dev/Autoware/ros/src/computing/perception/detection/packages/road_wizard/msg/ValueSet.msg" NAME_WE)
add_dependencies(road_wizard_generate_messages_py _road_wizard_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(road_wizard_genpy)
add_dependencies(road_wizard_genpy road_wizard_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS road_wizard_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/road_wizard
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(road_wizard_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(road_wizard_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/road_wizard
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(road_wizard_generate_messages_eus sensor_msgs_generate_messages_eus)
add_dependencies(road_wizard_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/road_wizard
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(road_wizard_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(road_wizard_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/road_wizard
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(road_wizard_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(road_wizard_generate_messages_py std_msgs_generate_messages_py)
