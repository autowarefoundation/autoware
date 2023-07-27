cmake_minimum_required(VERSION 3.5)
project(yabloc_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common kdtree)

# Sophus
find_package(Sophus REQUIRED)

# ===================================================
# Download DNN model
include(download.cmake)

# ===================================================
# Clear ${PYTHON_EXECUTABLE} defined by mrt_cmake_module so that rosidl_generate_interfaces can look for it properly
unset(PYTHON_EXECUTABLE)
message(STATUS "PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

# ===================================================
# Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SemanticSegmentation.srv"
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# ===================================================
# Executable
# Camera
set(TARGET camera_pose_initializer_node)
ament_auto_add_executable(${TARGET}
  src/camera/lane_image.cpp
  src/camera/marker_module.cpp
  src/camera/projector_module.cpp
  src/camera/camera_pose_initializer_core.cpp
  src/camera/camera_pose_initializer_node.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${PCL_LIBRARIES} Sophus::Sophus)

# ros2idl typesupport
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${TARGET} ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${TARGET} "${cpp_typesupport_target}")
endif()

# Semantic segmentation
install(PROGRAMS
  src/semantic_segmentation/semantic_segmentation_core.py
  src/semantic_segmentation/semantic_segmentation_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config data launch)
