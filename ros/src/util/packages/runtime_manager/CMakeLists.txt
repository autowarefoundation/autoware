cmake_minimum_required(VERSION 2.8.3)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
        rospy autoware_msgs autoware_config_msgs tablet_socket_msgs std_msgs
        )

catkin_package(
        CATKIN_DEPENDS rospy autoware_msgs autoware_config_msgs tablet_socket_msgs std_msgs
)

catkin_install_python(PROGRAMS
        scripts/ftrace.py
        scripts/proc_manager.py
        scripts/rtmgr.py
        scripts/runtime_manager_dialog.py
        scripts/test_pub.py
        scripts/test_srv.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
        scripts/3dm_gx5_15.sh
        scripts/add_perm.sh
        scripts/adis16470.sh
        scripts/avt_vimba.sh
        scripts/gazebo.sh
        scripts/mti300.sh
        scripts/vg440.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(FILES
        scripts/computing.yaml
        scripts/data.yaml
        scripts/empty.yaml
        scripts/interface.yaml
        scripts/main.yaml
        scripts/map.yaml
        scripts/qs.yaml
        scripts/rtmgr_icon.xpm
        scripts/rtmgr.wxg
        scripts/sensing.yaml
        scripts/setup.yaml
        scripts/simulation.yaml
        scripts/state.yaml
        scripts/status.yaml
        scripts/topics.yaml
        scripts/vector_map_files.yaml
        scripts/viewer.yaml
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/images
        #DESTINATION share/${PROJECT_NAME}/images
        )

install(FILES
        scripts/launch_files/check.launch
        scripts/launch_files/control.launch
        scripts/launch_files/init.launch
        scripts/launch_files/map.launch
        scripts/launch_files/perception.launch
        scripts/launch_files/planning.launch
        scripts/launch_files/sensor.launch
        scripts/launch_files/set.launch

        scripts/launch_files/velodyne_hdl64e_s2.launch
        scripts/launch_files/velodyne_hdl64e_s3.launch
        scripts/launch_files/velodyne_hdl32e.launch
        scripts/launch_files/velodyne_vlp16.launch
        scripts/launch_files/velodyne_vlp16_hires.launch
        scripts/launch_files/velodyne_vlp32c.launch

        scripts/vscan.launch
        scripts/vehicle_socket.launch
        scripts/uvc_camera.launch
        scripts/traffic_light.launch
        scripts/synchronization.launch
        scripts/tablet_socket.launch
        scripts/setup_tf.launch
        scripts/setup_vehicle_info.launch
        scripts/points2image.launch
        scripts/mqtt_socket.launch
        scripts/ex_mat_pub.launch
        scripts/calibration_publisher.launch
        scripts/avt_camera.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch_files
        #DESTINATION share/${PROJECT_NAME}/launch_files
        )


