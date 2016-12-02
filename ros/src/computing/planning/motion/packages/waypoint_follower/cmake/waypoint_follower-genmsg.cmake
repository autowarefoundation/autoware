# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "waypoint_follower: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iwaypoint_follower:/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(waypoint_follower_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg" NAME_WE)
add_custom_target(_waypoint_follower_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "waypoint_follower" "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg" ""
)

get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg" NAME_WE)
add_custom_target(_waypoint_follower_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "waypoint_follower" "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg" "geometry_msgs/TwistStamped:geometry_msgs/Point:geometry_msgs/Vector3:waypoint_follower/dtlane:geometry_msgs/Quaternion:geometry_msgs/Twist:geometry_msgs/Pose:geometry_msgs/PoseStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg" NAME_WE)
add_custom_target(_waypoint_follower_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "waypoint_follower" "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg" "geometry_msgs/TwistStamped:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Vector3:waypoint_follower/lane:std_msgs/Header:waypoint_follower/dtlane:geometry_msgs/Twist:waypoint_follower/waypoint:geometry_msgs/PoseStamped:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg" NAME_WE)
add_custom_target(_waypoint_follower_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "waypoint_follower" "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg" "geometry_msgs/TwistStamped:geometry_msgs/Point:geometry_msgs/Vector3:waypoint_follower/dtlane:geometry_msgs/Quaternion:geometry_msgs/Twist:waypoint_follower/waypoint:geometry_msgs/Pose:geometry_msgs/PoseStamped:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waypoint_follower
)
_generate_msg_cpp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waypoint_follower
)
_generate_msg_cpp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waypoint_follower
)
_generate_msg_cpp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waypoint_follower
)

### Generating Services

### Generating Module File
_generate_module_cpp(waypoint_follower
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waypoint_follower
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(waypoint_follower_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(waypoint_follower_generate_messages waypoint_follower_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_cpp _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_cpp _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_cpp _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_cpp _waypoint_follower_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waypoint_follower_gencpp)
add_dependencies(waypoint_follower_gencpp waypoint_follower_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waypoint_follower_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waypoint_follower
)
_generate_msg_eus(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waypoint_follower
)
_generate_msg_eus(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waypoint_follower
)
_generate_msg_eus(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waypoint_follower
)

### Generating Services

### Generating Module File
_generate_module_eus(waypoint_follower
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waypoint_follower
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(waypoint_follower_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(waypoint_follower_generate_messages waypoint_follower_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_eus _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_eus _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_eus _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_eus _waypoint_follower_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waypoint_follower_geneus)
add_dependencies(waypoint_follower_geneus waypoint_follower_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waypoint_follower_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waypoint_follower
)
_generate_msg_lisp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waypoint_follower
)
_generate_msg_lisp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waypoint_follower
)
_generate_msg_lisp(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waypoint_follower
)

### Generating Services

### Generating Module File
_generate_module_lisp(waypoint_follower
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waypoint_follower
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(waypoint_follower_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(waypoint_follower_generate_messages waypoint_follower_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_lisp _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_lisp _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_lisp _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_lisp _waypoint_follower_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waypoint_follower_genlisp)
add_dependencies(waypoint_follower_genlisp waypoint_follower_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waypoint_follower_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower
)
_generate_msg_py(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower
)
_generate_msg_py(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower
)
_generate_msg_py(waypoint_follower
  "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg;/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower
)

### Generating Services

### Generating Module File
_generate_module_py(waypoint_follower
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(waypoint_follower_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(waypoint_follower_generate_messages waypoint_follower_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/dtlane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_py _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/waypoint.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_py _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/LaneArray.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_py _waypoint_follower_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/Autoware/Autoware/ros/src/computing/planning/motion/packages/waypoint_follower/msg/lane.msg" NAME_WE)
add_dependencies(waypoint_follower_generate_messages_py _waypoint_follower_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waypoint_follower_genpy)
add_dependencies(waypoint_follower_genpy waypoint_follower_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waypoint_follower_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waypoint_follower)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waypoint_follower
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(waypoint_follower_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(waypoint_follower_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waypoint_follower)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waypoint_follower
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(waypoint_follower_generate_messages_eus std_msgs_generate_messages_eus)
add_dependencies(waypoint_follower_generate_messages_eus geometry_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waypoint_follower)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waypoint_follower
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(waypoint_follower_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(waypoint_follower_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waypoint_follower
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(waypoint_follower_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(waypoint_follower_generate_messages_py geometry_msgs_generate_messages_py)
