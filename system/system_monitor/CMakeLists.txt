cmake_minimum_required(VERSION 3.14)
project(system_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(NVML)
find_package(fmt REQUIRED)
set(LIBRARIES fmt)

## Specify additional locations of header files

find_path(LIBNL3_INCLUDE_DIRS
  NAMES netlink/netlink.h
  PATH_SUFFIXES libnl3
)

if(NVML_FOUND)
  include_directories(
    include
    ${LIBNL3_INCLUDE_DIRS}
    ${NVML_INCLUDE_DIRS}
  )
else()
  include_directories(
    include
    ${LIBNL3_INCLUDE_DIRS}
  )
endif()

## Declare a C++ executable

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(CMAKE_CPU_PLATFORM "intel")
  add_definitions(-D_CPU_INTEL_)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
  if(CMAKE_HOST_SYSTEM_VERSION MATCHES ".*raspi.*")
    set(CMAKE_CPU_PLATFORM "raspi")
    add_definitions(-D_CPU_RASPI_)
  elseif(CMAKE_HOST_SYSTEM_VERSION MATCHES ".*tegra.*")
    set(CMAKE_CPU_PLATFORM "tegra")
    add_definitions(-D_CPU_TEGRA_)
  else()
    set(CMAKE_CPU_PLATFORM "arm")
    add_definitions(-D_CPU_ARM_)
  endif()
else()
  set(CMAKE_CPU_PLATFORM "unknown")
endif()

if(NVML_FOUND)
  set(CMAKE_GPU_PLATFORM "nvml")
  add_definitions(-D_GPU_NVML_)
  set(GPU_LIBRARY ${NVML_LIBRARIES})
else()
  if(CMAKE_CPU_PLATFORM STREQUAL "tegra")
    set(CMAKE_GPU_PLATFORM "tegra")
    add_definitions(-D_GPU_TEGRA_)
  else()
    set(CMAKE_GPU_PLATFORM "unknown")
  endif()
endif()

message(STATUS "HOST_SYSTEM_VERSION: " ${CMAKE_HOST_SYSTEM_VERSION})
message(STATUS "SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR})
message(STATUS "CPU PLATFORM: " ${CMAKE_CPU_PLATFORM})
message(STATUS "GPU PLATFORM: " ${CMAKE_GPU_PLATFORM})

set(CPU_MONITOR_SOURCE
  src/cpu_monitor/cpu_monitor_base.cpp
  src/cpu_monitor/${CMAKE_CPU_PLATFORM}_cpu_monitor.cpp
)

ament_auto_add_library(cpu_monitor_lib SHARED
  ${CPU_MONITOR_SOURCE}
)

ament_auto_add_library(hdd_monitor_lib SHARED
  src/hdd_monitor/hdd_monitor.cpp
)

ament_auto_add_library(mem_monitor_lib SHARED
  src/mem_monitor/mem_monitor.cpp
)

ament_auto_add_library(net_monitor_lib SHARED
  src/net_monitor/net_monitor.cpp
  src/net_monitor/nl80211.cpp
)

ament_auto_add_library(ntp_monitor_lib SHARED
  src/ntp_monitor/ntp_monitor.cpp
)

ament_auto_add_library(process_monitor_lib SHARED
  src/process_monitor/process_monitor.cpp
)

set(GPU_MONITOR_SOURCE
  src/gpu_monitor/gpu_monitor_base.cpp
  src/gpu_monitor/${CMAKE_GPU_PLATFORM}_gpu_monitor.cpp
)
ament_auto_add_library(gpu_monitor_lib SHARED
  ${GPU_MONITOR_SOURCE}
)

ament_auto_add_library(voltage_monitor_lib SHARED
  src/voltage_monitor/voltage_monitor.cpp
)

ament_auto_add_executable(msr_reader
  reader/msr_reader/msr_reader.cpp
)

ament_auto_add_executable(hdd_reader
  reader/hdd_reader/hdd_reader.cpp
)

ament_auto_add_executable(traffic_reader
  reader/traffic_reader/traffic_reader_main.cpp
  reader/traffic_reader/traffic_reader_service.cpp
)

find_library(NL3 nl-3 REQUIRED)
find_library(NLGENL3 nl-genl-3 REQUIRED)
list(APPEND NL_LIBS ${NL3} ${NLGENL3})

find_package(Boost REQUIRED COMPONENTS
  serialization
  thread
  filesystem
  regex
)

## Specify libraries to link a library or executable target against
target_link_libraries(voltage_monitor_lib ${Boost_LIBRARIES} ${LIBRARIES})
target_link_libraries(cpu_monitor_lib ${Boost_LIBRARIES} ${LIBRARIES})
target_link_libraries(hdd_monitor_lib ${Boost_LIBRARIES} ${LIBRARIES})
target_link_libraries(mem_monitor_lib ${LIBRARIES})
target_link_libraries(net_monitor_lib ${NL_LIBS} ${LIBRARIES})
target_link_libraries(ntp_monitor_lib ${Boost_LIBRARIES} ${LIBRARIES})
target_link_libraries(process_monitor_lib ${LIBRARIES})
target_link_libraries(gpu_monitor_lib ${GPU_LIBRARY} ${Boost_LIBRARIES} ${LIBRARIES})
target_link_libraries(msr_reader ${Boost_LIBRARIES} ${LIBRARIES})
target_link_libraries(hdd_reader ${Boost_LIBRARIES} ${LIBRARIES})
target_link_libraries(traffic_reader ${Boost_LIBRARIES} ${LIBRARIES})

rclcpp_components_register_node(cpu_monitor_lib
  PLUGIN "CPUMonitor"
  EXECUTABLE cpu_monitor
)

rclcpp_components_register_node(hdd_monitor_lib
  PLUGIN "HddMonitor"
  EXECUTABLE hdd_monitor
)

rclcpp_components_register_node(mem_monitor_lib
  PLUGIN "MemMonitor"
  EXECUTABLE mem_monitor
)

rclcpp_components_register_node(net_monitor_lib
  PLUGIN "NetMonitor"
  EXECUTABLE net_monitor
)

rclcpp_components_register_node(ntp_monitor_lib
  PLUGIN "NTPMonitor"
  EXECUTABLE ntp_monitor
)

rclcpp_components_register_node(process_monitor_lib
  PLUGIN "ProcessMonitor"
  EXECUTABLE process_monitor
)

rclcpp_components_register_node(gpu_monitor_lib
  PLUGIN "GPUMonitor"
  EXECUTABLE gpu_monitor
)

rclcpp_components_register_node(voltage_monitor_lib
  PLUGIN "VoltageMonitor"
  EXECUTABLE voltage_monitor
)

# TODO(yunus.caliskan): Port the tests to ROS 2, robustify the tests.
if(BUILD_TESTING)
  # ament_add_ros_isolated_gtest(test_cpu_monitor
  #   test/src/cpu_monitor/test_${CMAKE_CPU_PLATFORM}_cpu_monitor.cpp
  #   ${CPU_MONITOR_SOURCE}
  # )

  # ament_target_dependencies(test_cpu_monitor
  #   "rclcpp"
  #   "diagnostic_msgs"
  # )

  # target_include_directories(test_cpu_monitor
  #   PRIVATE "include"
  # )

  # target_link_libraries(test_cpu_monitor ${Boost_LIBRARIES} ${LIBRARIES})

  # ament_add_ros_isolated_gtest(test_hdd_monitor
  #   test/src/hdd_monitor/test_hdd_monitor.cpp
  #   src/hdd_monitor/hdd_monitor.cpp
  # )

  # ament_target_dependencies(test_hdd_monitor
  #   "rclcpp"
  #   "diagnostic_msgs"
  # )

  # target_include_directories(test_hdd_monitor
  #   PRIVATE "include"
  # )

  # target_link_libraries(test_hdd_monitor ${Boost_LIBRARIES} ${LIBRARIES}
  # )

  # ament_add_ros_isolated_gtest(test_mem_monitor
  #   test/src/mem_monitor/test_mem_monitor.cpp
  #   src/mem_monitor/mem_monitor.cpp
  # )

  # ament_target_dependencies(test_mem_monitor
  #   "rclcpp"
  #   "diagnostic_msgs"
  # )

  # target_include_directories(test_mem_monitor
  #   PRIVATE "include"
  # )

  # target_link_libraries(test_mem_monitor ${Boost_LIBRARIES} ${LIBRARIES})

  # ament_add_ros_isolated_gtest(test_net_monitor
  #   test/src/net_monitor/test_net_monitor.cpp
  #   src/net_monitor/net_monitor.cpp
  #   src/net_monitor/nl80211.cpp
  # )

  # ament_target_dependencies(test_net_monitor
  #   "rclcpp"
  #   "diagnostic_msgs"
  # )

  # target_include_directories(test_net_monitor
  #   PRIVATE "include"
  # )

  # target_link_libraries(test_net_monitor ${Boost_LIBRARIES} ${NL_LIBS} ${LIBRARIES})

  # ament_add_ros_isolated_gtest(test_ntp_monitor
  #   test/src/ntp_monitor/test_ntp_monitor.cpp
  #   src/ntp_monitor/ntp_monitor.cpp
  # )

  # ament_target_dependencies(test_ntp_monitor
  #   "rclcpp"
  #   "diagnostic_msgs"
  # )

  # target_include_directories(test_ntp_monitor
  #   PRIVATE "include"
  # )

  # target_link_libraries(test_ntp_monitor ${Boost_LIBRARIES} ${LIBRARIES})

  # ament_add_ros_isolated_gtest(test_process_monitor
  #   test/src/process_monitor/test_process_monitor.cpp
  #   src/process_monitor/process_monitor.cpp
  # )

  # ament_target_dependencies(test_process_monitor
  #   "rclcpp"
  #   "diagnostic_msgs"
  # )

  # target_include_directories(test_process_monitor
  #   PRIVATE "include"
  # )

  # target_link_libraries(test_process_monitor ${Boost_LIBRARIES} ${LIBRARIES})

  # ament_add_ros_isolated_gtest(test_gpu_monitor
  #   test/src/gpu_monitor/test_${CMAKE_GPU_PLATFORM}_gpu_monitor.cpp
  #   ${GPU_MONITOR_SOURCE}
  # )

  # ament_target_dependencies(test_gpu_monitor
  #   "rclcpp"
  #   "diagnostic_msgs"
  # )

  # target_include_directories(test_gpu_monitor
  #   PRIVATE "include"
  # )

  # target_link_libraries(test_gpu_monitor ${GPU_LIBRARY} ${LIBRARIES})

endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
