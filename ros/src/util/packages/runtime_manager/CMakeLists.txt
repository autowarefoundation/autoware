cmake_minimum_required(VERSION 2.8.3)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
        rospy
        )

catkin_install_python(PROGRAMS
  scripts/ftrace.py
  scripts/proc_manager.py
  scripts/rtmgr.py
  scripts/runtime_manager_dialog.py
  scripts/test_pub.py
  scripts/test_srv.py
  DESTINATION share/${PROJECT_NAME})

install(FILES
  scripts/3dm_gx5_15.sh
  scripts/add_perm.sh
  scripts/adis16470.sh
  scripts/avt_camera.launch
  scripts/avt_vimba.sh
  scripts/calibration_publisher.launch
  scripts/computing.yaml
  scripts/data.yaml
  scripts/empty.yaml
  scripts/ex_mat_pub.launch
  scripts/gazebo.sh
  scripts/interface.yaml
  scripts/main.yaml
  scripts/map.yaml
  scripts/mqtt_socket.launch
  scripts/mti300.sh
  scripts/points2image.launch
  scripts/qs.yaml
  scripts/rtmgr_icon.xpm
  scripts/rtmgr.wxg
  scripts/sensing.yaml
  scripts/setup_tf.launch
  scripts/setup_vehicle_info.launch
  scripts/setup.yaml
  scripts/simulation.yaml
  scripts/state.yaml
  scripts/status.yaml
  scripts/synchronization.launch
  scripts/tablet_socket.launch
  scripts/topics.yaml
  scripts/traffic_light.launch
  scripts/uvc_camera.launch
  scripts/vector_map_files.yaml
  scripts/vehicle_socket.launch
  scripts/vg440.sh
  scripts/viewer.yaml
  scripts/vscan.launch
  DESTINATION share/${PROJECT_NAME})

install(FILES
  scripts/images/autoware_logo_1.png
  scripts/images/autoware_logo_2.png
  scripts/images/autoware_logo_2_white.png
  scripts/images/axe.png
  scripts/images/database.png
  scripts/images/dec.png
  scripts/images/inc.png
  scripts/images/mobile.png
  scripts/images/nagoya_university.png
  scripts/images/tablet.png
  scripts/images/vehicle.png
  DESTINATION share/${PROJECT_NAME}/images)

install(FILES
  scripts/launch_files/check.launch
  scripts/launch_files/control.launch
  scripts/launch_files/init.launch
  scripts/launch_files/map.launch
  scripts/launch_files/perception.launch
  scripts/launch_files/planning.launch
  scripts/launch_files/sensor.launch
  scripts/launch_files/set.launch
  DESTINATION share/${PROJECT_NAME}/launch_files)

catkin_package(
        CATKIN_DEPENDS rospy std_msgs message_runtime
)

