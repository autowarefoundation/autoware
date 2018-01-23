^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package runtime_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge pull request `#878 <https://github.com/CPFL/Autoware/issues/878>`_ from CPFL/feature/ndt_pcl_gpu
  Feature/ndt pcl gpu
* Merge branch develop into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/tlr_turn_support
* Merge pull request `#954 <https://github.com/CPFL/Autoware/issues/954>`_ from CPFL/fix/tf_mapping
  Fix/tf mapping
* add new state button on runtime manager
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* Merge branch 'develop' into feature/tlr_turn
* Merge branch 'develop' into feature/ndt_pcl_gpu
* add new state for gui
* Merge pull request `#936 <https://github.com/CPFL/Autoware/issues/936>`_ from CPFL/feature/decision
  Feature/decision: Enhancement decision maker node
* Added support to Image Publisher for frames different than only "velodyne". (`#946 <https://github.com/CPFL/Autoware/issues/946>`_)
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* Merge branch 'feature/OpenPlanner' into develop
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'develop' of github.com:CPFL/Autoware into feature/decision
* [fix] Image src correctly set in the config file of runtime manager for ssd node (`#939 <https://github.com/CPFL/Autoware/issues/939>`_)
* Fixed misalignment on state tab with Ubuntu14.04
* tweak state tab
* update and add state for gui
* update gui
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* Fixed:
  - callback
  - laneshift
  Added:
  - publisher for laneid
  - new lanechange flag
  - new param for decisionMaker
* add to insert shift lane
* update state and remove detection state
* change state button
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* - Add user interface option in runtime manage for local_planner and op_trajectory_generator
* fix computing.yaml
* add app dialog for fix2tfpose and nmea2tfpose (WIP)
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Fix feature/points2image bug multicam support (`#886 <https://github.com/CPFL/Autoware/issues/886>`_)
  * pointgrey
  * Added New Calibration node
  * Added parameters, plane fitting
  * added mirror node, etc
  * Points2Image
  Calibration Publisher
  now works with multiple cameras using ros namespaces
  * Including only points2image
  * Added Launch file for points2 image specific for the ladybug camera
* [feature] Added NDT Mapping Descriptions and checkboxes to Runtime Manager UI (`#882 <https://github.com/CPFL/Autoware/issues/882>`_)
  * Added Descriptions and checkboxes to Runtime Manager UI
  * Added also approximate ndt mapping descriptions
* remove unnecessary gui
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge branch 'develop' into feature/ndt_pcl_gpu
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/CMakeLists.txt
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_mapping/ndt_mapping.cpp
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Add ndt_cpu library
* Merge pull request `#844 <https://github.com/CPFL/Autoware/issues/844>`_ from CPFL/feature/remote_monitor
  Feature/remote monitor
* add path velocity smoothing
* add waypoint relay
* Runtime manager updated to new package names (`#870 <https://github.com/CPFL/Autoware/issues/870>`_)
  [fix] Runtime manager updated to new pgrey package names
* Merge branch 'master' into feature/decision
* Feature/fusion_filter - fusion multiple lidar (`#842 <https://github.com/CPFL/Autoware/issues/842>`_)
  * Add fusion_filter to merge multiple lidar pointclouds
  * Refactor fusion_filter
  * Apply clang-format and rebase develop
  * Add fusion_filter launch and runtime_manager config
  * Fix names, fusion_filter -> points_concat_filter
  * Fix build error in ros-indigo
  * Fix some default message/frame names
  * Refactor code and apply clang-format
  * Add configrations for runtime_manager
  * Fix CMake
* Feature/cloud transformer (`#860 <https://github.com/CPFL/Autoware/issues/860>`_)
  * Added Cloud transformer node
  transforms pointcloud to a target frame
  * Added support for XYZIR point type
  * Added error checks when transformation unavailable
* Merge branch 'master' of github.com:cpfl/autoware into develop
* add commonly buttons
* rename state tabs
* add select state tab
* moved images
* Merge pull request `#794 <https://github.com/CPFL/Autoware/issues/794>`_ from CPFL/feature/ndt_pcl_gpu
  Feature/ndt pcl gpu
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* add use_gpu option for ndt_matching and ndt_mapping in app dialog
* Merge branch 'develop' of github.com:CPFL/Autoware into feature/remote_monitor
* Merge pull request `#833 <https://github.com/CPFL/Autoware/issues/833>`_ from CPFL/feature/rtmgr_fix_checkbox_tree
  Runtime Manager, fix wrong scrolling when checked on kinetic
* Runtime Manager, fix wrong scrolling when checked on kinetic and hlink layout
* Merge for ndt_pcl_gpu
* Merge pull request `#831 <https://github.com/CPFL/Autoware/issues/831>`_ from CPFL/feature/rtmgr_update_rosbag_rec
  Runtime Manager, rosbag filename update when ROSBAG button on
* merge develop
* Runtime Manager, rosbag filename update when ROSBAG button on, and fix unnecessary ext.
* Merge pull request `#826 <https://github.com/CPFL/Autoware/issues/826>`_ from CPFL/feature/ray_ground_filter
  Feature/ray ground filter
* Merge branch 'develop' into feature/ray_ground_filter
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/sensing.yaml
* Add ground_filter config for runtime_manager (`#828 <https://github.com/CPFL/Autoware/issues/828>`_)
* Added descriptions to the params in launch file
* Ray Ground Filter Initial Commit
* merge develop
* add approximate_ndt_mapping (`#811 <https://github.com/CPFL/Autoware/issues/811>`_)
* fix bug and refactor code
* Merge branch 'feature/decision_maker' of github.com:cpfl/autoware into feature/remote_monitor
* add new button for remote monitor to runtime_manager
* Contributors: AMC, Abraham Monrroy, Akihito Ohsato, Yamato ANDO, YamatoAndo, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, anhnv-3991, hatem-darweesh, kondoh, kondoh2, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* Merge pull request `#808 <https://github.com/cpfl/autoware/issues/808>`_ from CPFL/feature/decision_maker
  [WIP]Feature/decision maker `#807 <https://github.com/cpfl/autoware/issues/807>`_
* update decision maker config
* Add to support dynamical parameter for decision_maker
* Change default value of decision maker config
* Add decision_maker config
* Runtime Manager, modify update_func()
* fix msg import for planner_switch
* add config parameter
* Runtime Manager Sensing tab, remove calibrationfile check of GH3 camera
* Add decision packages into runtime_manager
* Trash
* Add changing topic name option for the planner selector.
* add multiple crosswalk detection
* Add parameter dialog in runtime manager
* support Garmin GPS 18x LVC
* Add vector_map_server to RuntimeManager  `#722 <https://github.com/cpfl/autoware/issues/722>`_
* Contributors: TomohitoAndo, Yusuke FUJII, andoh104, kondoh, yukikitsukawa

1.4.0 (2017-08-04)
------------------
* Publish initial light color as green
* Contributors: TomohitoAndo

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix a build issue due to autoware_msgs on the Indigo
* Resolved merge conflict by new feature
* Merge branch 'develop' into feature/TLR_by_machine_learning
* Merge pull request `#721 <https://github.com/CPFL/Autoware/issues/721>`_ from CPFL/feature/astar_planner
  Feature/astar planner
* Add obstacle_avoid parameters
* Add parameters for points2costmap
* Add checkbox for dist_transform node in Computing tab
* Add selecter of GPU for euclidean clustering
* Runtime Manager, update for showing of use memory
* Prepare for merge
  * Fix assumed SSD path in CMakeLists.txt
  * Change default path of trained model into package-internal directory
  * Remove `std::cerr` statements for debug
  * Add UI to boot `traffic_light_recognition_ssd.launch` from runtime-manager
* convert to autoware_msgs
* add checkbox for MicroStrain 3DM-GX5-15 driver in Sensing tab
* Contributors: Manato Hirabayashi, TomohitoAndo, YamatoAndo, Yusuke FUJII, kondoh, yukikitsukawa, yukitsuji

1.2.0 (2017-06-07)
------------------
* Merge pull request `#669 <https://github.com/CPFL/Autoware/issues/669>`_ from CPFL/feature/yolo2
  Feature/yolo2
* Merge pull request `#671 <https://github.com/CPFL/Autoware/issues/671>`_ from CPFL/feature/ndt_imu
  Feature/ndt imu
* Added Video to Main Readme, added in Runtime Manager
* Merge remote-tracking branch 'origin/develop' into feature/yolo2
* Yolo 2 Node completed
* add can2odom.launch
* Change OpenPlanner stand alone libraries names.
* Merge branch 'develop' into feature/lidar_segmentation
* Merge pull request `#618 <https://github.com/CPFL/Autoware/issues/618>`_ from CPFL/feature/rtmgr_booted_cmds
  Runtime Manager, add booted_cmd to param.yaml and select dialog at boot
* Add OpenPlanner to Develop Branch, add OpenPlanner to Runtime Manager, and modify rviz default config file
  fix map loading options
  automatic replanning simulation and traffic light stop and go
  add performance logging
  behavior state for traffic light and stop signs fixed
  fix logging shift, fix euclidean clusters problem
  visualize dp steps
  detection config for robot vel16
  tune ff path follower for simulation
  tune ff path follower for simulation
  HMI update
  simulated obstacle bounding box representation
  HMI Update
  HMI Successful Demo
  improve detection accuracy to < 10 cm
  HMI Tested. More runtime manager options.
  HMI Tested. More runtime manager options.
  fix dp plan build issue
  Controller - Steering Delay auto calibration
  Multi-Traffic Behavior Simulation on Rviz using OpenPlanner
  change node names to match ROS naming standards
  change node names to match ROS naming standards
  - Add OpenPlanner Vehicle Simulator
  - Integrate with Autoware's pure pursut
  - Revised local planning
  - Unit-Test usig playback based simulation
  update simulation launch files
  More Unit Testing
  Improve Object Tracking
  CAN info message handle!
  rviz config
  visualization changes
  add option to select velocities source
  RS Planner Test
* Runtime Manager Sensing tab, move mti300.launch from runtime_manger to xsens_driver
* Runtime Manager Sensing tab, fix typo mti300.launch
* Runtime Manager Sensing tab, add sleep at booting between mtdevice and mtnode
* Runtime Manager, update add_perm.sh for gksudo cancel button
* Runtime Manager, changed from sudo to gksudo in add_perm.sh
* Runtime Manager Sensing tab, add mti300.sh and mti300.launch for Xsens MTi-300
* Runtime Manager Sensing tab, add param for MTi-300
* Runtime Manager Sensing tab, add Xsens MTi-300 entry
* Runtime Manager Sensing tab, add vg440.sh and add_perm.sh for Memsic VG440
* Runtime Manager, add do_shell_exec keyword for shell=True arg in Popen()
* Runtime Manager Sensing tab, add param for Memsic VG440
* Runtime Manager Sensing tab, rename from Crossbow vg440 to Memsic VG440
* Merge pull request `#637 <https://github.com/CPFL/Autoware/issues/637>`_ from CPFL/feature/kinetic
  Add Kinetic support
  resolve `#503 <https://github.com/CPFL/Autoware/issues/503>`_
* add psutil 3.x to 1.x backward compatibility
* add measurement_range
  refactoring
* Runtime Manager, fix error at calibration_publisher button off
* Merge branch 'develop' into feature/lidar_segmentation
* Merge pull request `#621 <https://github.com/CPFL/Autoware/issues/621>`_ from CPFL/feature/rtmgr_for_wrong_scroll
  Runtime Manager, update for tree checkbox, for wrong scrolling at che…
* follow api change in psutil
* fix runtime_manager dependency (on tablet_socket_msgs)
* Update for kinetic
* Fixed an indication in review.
* Reflected the results of the review.
* Added sys dialog of ssd node.
* Runtime Manager, update for tree checkbox, for wrong scrolling at checked
* Runtime Manager, add Enable/Disable booted_cmds to quit_select
* fix, remove boot_cmds at no booted cmds, and add using label
* Runtime Manager, add booted_cmd to param.yaml and select dialog at boot
* Merge pull request `#616 <https://github.com/CPFL/Autoware/issues/616>`_ from CPFL/feature/rtmgr_quit_select
  Runtime Manager, add quit_select() and reload_computing_yaml()
* Runtime Manager, add proc_wait_thread() for async kill proc
* Runtime Manager, add quit_select() and reload_computing_yaml()
* Merge remote-tracking branch 'origin/develop' into feature/lidar_segmentation
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Kf Added
  Euclidean Cluster improved
* Fixes
* Merge branch 'master' into lidar_segmentation
* Merge branch 'master' into lidar_segmentation
  Conflicts:
  ros/src/computing/perception/detection/packages/lidar_tracker/nodes/euclidean_cluster/euclidean_cluster.cpp
* Added SVM python node
  Merge branch 'master' into lidar_segmentation
  Conflicts:
  ros/src/computing/perception/detection/packages/lidar_tracker/launch/euclidean_clustering.launch
  ros/src/computing/perception/detection/packages/lidar_tracker/nodes/euclidean_cluster/euclidean_cluster.cpp
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'master' into lidar_segmentation
* Merge branch 'master' into lidar_segmentation
* Merge branch 'master' into lidar_segmentation
* Merge branch 'master' into lidar_segmentation
* Added VectorMap Server support
* Lidar tracker restructuration
* Added output_frame param
  Allows to transform output coordinate frame of the bounding boxes and CloudClusterArray messages
* Merge remote-tracking branch 'origin/master' into lidar_segmentation
* Added Ground removal as optional.
  Removed unused params
* Parametrization of Clustering params
* Contributors: AMC, Shohei Fujii, Yukihiro Saito, Yusuke FUJII, hatem-darweesh, kondoh, kondoh2, tange, yukikitsukawa

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Fix app display for vel_pose_connect
* Merge pull request `#599 <https://github.com/CPFL/Autoware/issues/599>`_ from CPFL/feature/add_max_iterations
  Feature/add max iterations
* size adjustment of config window
* add set_size_gdic() for adjust dialog size
* add max_iterations, min_scan_range and min_add_scan_shift
* Merge pull request `#592 <https://github.com/CPFL/Autoware/issues/592>`_ from CPFL/feature/autoware-connector
  Add autoware_connector instead of vel_pose_mux
* add max_iterations
* Merge branch 'develop' into fix/map_tools
* Runtime Manager Map tab, add Point Type to PCD Filter and PCD Binarizer
* Add euclidean lidar track
* Merge branch 'develop' into feature/update_obj_fusion
* Add dummy tracking node
* Add autoware_connector instead of vel_pose_mux
* Merge pull request `#590 <https://github.com/CPFL/Autoware/issues/590>`_ from CPFL/feature/modify-lane-select
  Feature/modify lane select
* Fix computing.yaml
* Change Parameter range and initial value
* parameter from runtime manager
* Merge pull request `#564 <https://github.com/CPFL/Autoware/issues/564>`_ from CPFL/feature/twist_filter
  Feature/twist filter
* Update sensing.yaml
* Merge branch 'develop' into feature/simulator-with-errors
* Merge branch 'develop' into feature/twist_filter
* Add param bar of twist filter node in runtime manager
* Merge branch 'develop' into feature/state-machine
* add Gazebo button on Simulation tab
* Use integer size with temporal waypoints
* Merge branch 'develop' into feature/potential_field
* Add state_machine on runtime manager
* Merge pull request `#571 <https://github.com/CPFL/Autoware/issues/571>`_ from CPFL/feature/refactor_velocity_set
  Feature/refactor velocity set
* New simulator with angle and position errors
* Change minimun and maximum value of parameters of pure pursuit
* Change the default parameter
* Merge pull request `#568 <https://github.com/CPFL/Autoware/issues/568>`_ from CPFL/feature/dev-waypoint-follower
  Bug fix for linear interpolation flag and command velocity
* Bug fix for linear interpolation flag and command velocity
* Merge pull request `#567 <https://github.com/CPFL/Autoware/issues/567>`_ from CPFL/feature/auto-lane-change
  Feature/auto lane change
* Add potential field in runtime manager
* Add topic publisher for steering robot
* Change parameter name in app tab
* Rewrite to change local planning to global planning
* Edit computing.yaml for lane_select and fix typo
* Add support for multi lane files
* Contributors: Hiroki Ohta, TomohitoAndo, Yuki Kitsukawa, Yukihiro Saito, h_ohta, kondoh, pdsljp, yukikitsukawa

1.0.1 (2017-01-14)
------------------
* Fix camera_ids.
* Contributors: USUDA Hisashi

1.0.0 (2016-12-22)
------------------
* Add dialog in Runtime Manager
* Runtime Manager Computing tab, fix euclidean_cluster default data
* Modify to launch the node in Runtime Manager
* Merge pull request `#523 <https://github.com/CPFL/Autoware/issues/523>`_ from CPFL/fix_lane_stop_checkbox
  add checkbox value of lane_stop parameter to pdic
* add checkbox value of lane_stop parameter to pdic
* Added param to ignore points closer than a threshold
* Fix for using the twist of MKZ as current velocity
* Create red and green lanes from waypoint_saver CSV
* Node Name Change for cv tracker
* Added SSD node to CV Tracker
* Rename variables.
* Lidar segmentation (`#499 <https://github.com/CPFL/Autoware/issues/499>`_)
  * Lidar tracker restructuration
  * Added points_preprocessor package, including; ground filtering and space filtering.
* Separate motion planning package
* Merge pull request `#491 <https://github.com/CPFL/Autoware/issues/491>`_ from CPFL/add-get-height
  Add get height
  If 'Get Height' checkbox is enabled on ndt_matching, you get height of initial pose by 2D Pose Estimate.
  This is height of nearest point in /points_map.
* Add get height
  If 'Get Height' checkbox is enabled on ndt_matching, you get height of initial pose by 2D Pose Estimate.
  This is height of nearest point in /points_map.
* Added output_frame param
  Allows to transform output coordinate frame of the bounding boxes and CloudClusterArray messages
* Lidar segmentation (`#486 <https://github.com/CPFL/Autoware/issues/486>`_)
  Added CloudCluster and CloudClusterArray Message
  Clusters and its feats can now be accessed from outside the clustering node.
  Refer to the messages definition
* Update the check.launch for the tablet_socket because the node ndt_pcl change to ndt_matching
* Add parameter for subscribing topic
* Lidar segmentation (`#482 <https://github.com/CPFL/Autoware/issues/482>`_)
  * Added Cluster class
  * Parametrization of Clustering params
* Added params for Cloud clipping
  fixed bug in segment by distance
* Added
  RuntimeManager control for Euclidean clustering
  Distance based threshold for clusteringd
* Fix bug for changing the name of variables
* Runtime Manager Simulation tab, add rosbag info thread
* Change variable name in ConfigWaypointFollower, calculate function for lookahead distance
* Define vehicle acceleration
* Fix bug for changing the name of variables
* Runtime Manager Simulation tab, add rosbag info thread
* Merge pull request `#467 <https://github.com/CPFL/Autoware/issues/467>`_ from CPFL/develop-planner
  Develop planner
* Runtime Manager Map tab, update to more simple implements
* Runtime Manager Computing tab, add use_crosswalk_detection to velocity_set
* Remove unused parameters
* Change variable name in ConfigWaypointFollower, calculate function for lookahead distance
* Runtime Manager, changed wx.Color to wx.Colour for wxPython3
* Runtime Manager, fix rosbag record stop
* Merge branch 'add_ftrace'
  Conflicts:
  ros/src/.config/rviz/cmd.sh
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
* Rewrite comment
  translate comment message from Japanese to English.
* Modify ftrace scale display
  modify to display ftrace scale.
* Modify continuous ftrace display
  modify to display continuous ftrace status.
* Define vehicle acceleration
* Runtime Manager, delete MyFrame.OnTextRoute() MyFrame.OnRef(), not referenced from anywhere
* Runtime Manager, delete commented-out functions
* Runtime Manager, change button_launch_xxx to button_xxx
* Runtime Manager, delete OnLaunc_obj() OnKill_kill_obj(), not referenced from anywhere
* Runtime Manager, update OnLink() in dialog class to use frame.button_xxx
* Runtime Manager, update OnSelector_name() to use OnLaunchKill_obj()
* Runtime Manager, add button_xxx StrValObj in setup_buttons() when not found glade setting
* Runtime Manager, delete OnLaunch(), OnKill(), not referenced from anywhere
* Runtime Manager, delete kill_all(),kill_proc(),kill_obj(), simplified at OnClose()
* Runtime Manager, add StrValObj.SetValue()
* Runtime Manager, delete nodes_dic, not referenced from anywhere
* Runtime Manager, delete all_proc_nodes, not referenced from anywhere
* Runtime Manager, delete is_boot() is nosense, all_procs_nodes made from nodes_dic
* Runtime Manager, delete MyFfame.OnRefresh(), not called from anywhere
* Runtime Manager, add kill_children flag to RViz cmd for remote termination
* Runtime Manager Setup tab, add Vehicle Info
* Runtime Manager, terminate_children() changed to as option
* Runtime Manager, delete SIGTERM flag from Detection in Quick Start tab
* Runtime Manager, update tablet_sender/receiver for SIGINT termination
* Runtime Manager, update points_map_loader for SIGINT termination
* Runtime Manager, update feat_proj for SIGINT termination
* Runtime Manager, change SIGTERM to SIGINT at cmd termination
* Accomodate to vel_pose_mux
* Runtime Manager, fix getting proc info in ParamPanel class
* Runtime Manager, update some utility functions
* Runtime Manager Map tab, fix toggle enable of (Point Cloud) btn
* Runtime Manager Sensing tab, update filter cmd for Synchronization button
* Runtime Manger, fix camera_id setting at sync on
* Speed up using set_ftrace_pid
* Runtime Manager, fix camera_id selection at Synchronization ON
* Runtime Manager Simulation tab, fix showing of rosbag pos remains on stop
* Runtime Manager Topics tab, fix for alias of bottom btns
* Runtime Manager Quick Start tab, fix Map load OK label
* Runtime Manager Computing tab, add use_openmp arg to ndt
* Add module graph tool
* Merge pull request `#343 <https://github.com/CPFL/Autoware/issues/343>`_ from CPFL/add_desc
  Runtime Manager, Add desc
* add obj_enables utils for widget enable/disable
* refactoring config_dic search, add param sys
* Runtime Manager, add utils for dict and list
* Add lazy_ndt_mapping.
  Add checkbox for lazy_ndt_mapping in Computing tab.
* Merge pull request `#323 <https://github.com/CPFL/Autoware/issues/323>`_ from CPFL/add_icp_localizer
  Add icp_localizer.
* Add checkbox of icp_matching to Computing tab.
  Add ConfigICP.msg.
* Runtime Manager Map tab, add 1GB check for .pcd files
* Runtime Manager, fix Gdk warning at getting tab names
* Runtime Manager, add desc of ROSBAG button in quick start tab
* Runtime Manager, add desc to quick start alias buttons
* Runtime Manager, add tooltip utility function
* Change label using ros node name instead of process id.
* Runtime Manger, add desc sample to computing.yaml and sensing.yaml
* Runtime Manager, update desc for mouse hover
* Runtime Manager Computing tab, add description
* Runtime Manager Computing tab, fix typo points2costmap
* Remove unnecessary parameters from config window of ndt_matching.
* Runtime Manager Computing tab, add Semantics package
* Show UVC camera ID
  If clicking USB Generic checkbox, regard UVC camera ID as /camera0.
* Add icp_localizer package.
* Merge pull request `#308 <https://github.com/CPFL/Autoware/issues/308>`_ from CPFL/integrate_filter
  Separate downsampling of points_raw from ndt_matching.
* Changed directory structure.
  Add PointsFilterInfo.msg.
  Modified to publish points_filter_info.
* Runtime Manager, Points Filter moved to Sensing tab
* Add config window of each filter in Computing tab.
  Add message files for each filter.
  Modified to support dynamic parameter tuning for each filter.
* Runtime Manager Computing tab, fix for sync option problem
* Bug fix of distance_filter.
  Add random_filter.
  Modified ndt_matching to subscribe /filtered_points instead of /points_raw.
* Separate downsampling process of scan data from ndt_matching.
* Fix initialize value
* Modify ftrace
  - Show name by tooltip
  - Read ftrace info by proc_manager
* Fix some parts
* Remove sim_mode
* Add vel_pose_mux package
* Extract two function from duplicate part ,Change to select next target from next waypoint if next waypoint is first or last
* Change to set Initial Pose from TF, if initial source is localizer or gnss
* Runtime Manager, fix lane_stop dialog button color
* Runtime Manager, update for immediately reflection of sys CPU setup
* Runtime Manager, update for remote rviz
* Runtime Manager, update to use gdic(gui: in .yaml) at no param
* fix save param check, not include sys_prm for check
* Runtime Manager Setup tab, fix buttons setting
* fix for float("123,456"), add str_to_float()
* Add Ftrace at Status tab of Runtime Manager.
  各ノードの動作状況をftraceで取得して表示しますが、いろいろ足りていません。
  - gksudo経由で実行します。(Runtime Managerからの終了ができません。)
  - ノード名ではなくプロセスIDで表示されます。
  - 横軸(時間)がありません。
  - リアルタイムに表示されません。
* Runtime Manager Setup tab, parameter model_path changed to fullpath
* Fix choices in velocity source
* Runtime Manager, workaround for file save dialog segfault
* Redesign map_downloader dialog
* Runtime Manager, support text control passwd flag
* Modify to get camera ID from runtime manager
  * Make it possible to obtain camera ID in each node to subscribe topics
  under camera name space selectively
  * Get image size from sensor_msgs::Image instead of CameraInfo
* Runtime Manager, resolved conflicts apply_multi_camera
* Merge branch 'master' of ssh://localhost:8443/CPFL/Autoware into apply_multi_camera
  WARNING: This commit contains unresolved conflict.
  Unresolved conflicts are contained in...:
  *
  ros/src/util/packages/runtime_manager/scripts/runtime_manager_dialog.py
  * ros/src/util/packages/runtime_manager/scripts/computing.yaml
  * ros/src/util/packages/runtime_manager/scripts/sensing.yaml
  Conflicts:
  ros/src/computing/perception/detection/packages/cv_tracker/launch/dpm_ocv.launch
  ros/src/computing/perception/detection/packages/cv_tracker/launch/dpm_ttic.launch
  ros/src/computing/perception/detection/packages/cv_tracker/launch/kf_tracking.launch
  ros/src/computing/perception/detection/packages/cv_tracker/launch/reprojection.launch
  ros/src/computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/obj_reproj.cpp
  ros/src/computing/perception/detection/packages/viewers/nodes/scan_image_viewer/scan_image_viewer.cpp
  ros/src/sensing/fusion/packages/calibration_camera_lidar/CalibrationToolkit/calibrationtoolkit.cpp
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
  ros/src/util/packages/runtime_manager/scripts/points2image.launch
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
  ros/src/util/packages/runtime_manager/scripts/runtime_manager_dialog.py
  ros/src/util/packages/runtime_manager/scripts/sensing.yaml
  ros/src/util/packages/runtime_manager/scripts/vscan.launch
* Runtime Manager, fix dialog showing after closeing by close box
* Removed *.orig files
* Merge remote-tracking branch 'origin/master' into synchrogazed
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/launch/ndt_matching.launch
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Change a subscribing topic  in points2image and vscan2points when synchronization is enabled
* Runtime Manager Setup tab, move rosparam tf_xxx setting from setup_tf.launch
* Runtime Manager Setup tab, add enable/disable toggle to radio box
* Runtime Manager Setup tab, add localizer radio box
* Merge remote-tracking branch 'origin/master' into synchrogazed
  Conflicts:
  ros/src/sensing/fusion/packages/scan2image/CMakeLists.txt
* Merge remote-tracking branch 'origin/fix_timestamp' into synchrogazed
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
* Merge pull request `#231 <https://github.com/CPFL/Autoware/issues/231>`_ from CPFL/fix-pure-pursuit
  Fix pure pursuit
* Change initial value
* Merge remote-tracking branch 'origin/resolve_conflict' into synchrogazed
  Conflicts:
  ros/src/computing/perception/detection/packages/cv_tracker/launch/ranging.launch
  ros/src/computing/perception/detection/packages/cv_tracker/nodes/obj_reproj/obj_reproj.cpp
  ros/src/util/packages/runtime_manager/scripts/rtmgr.py
  ros/src/util/packages/runtime_manager/scripts/rtmgr.wxg
  ros/src/util/packages/runtime_manager/scripts/runtime_manager_dialog.py
* Publish ConfigLaneStop message
* Rename /traffic_light topic
* Runtime Manager Computing tab, modify lane_stop gui handler name
* Runtime Manager Computing tab, update lane_stop GUI
* Runtime Manager, modify window title
* Send shutdown signal to process manager on exit
* Return error value at unknown operation key receieved
* Add linear interpolate mode Switch
* Runtime Manager, fix func overwrite at dialog close
* Runtime Manager Computing tab, fix obj_reproj, obj_fusion launch
* Runtime Manager Computing tab, fix obj_reproj launch
* Runtime Manager Computing tab, fix obj_reproj, obj_fusion launch
* Runtime Manager Computing tab, add Synchronization button
* merged master `#123 <https://github.com/CPFL/Autoware/issues/123>`_, for lost cmd args
* Runtime Manager, add select camera dialog
* Runtime Manager, update Calibration Publisher for multi cameras
* Add function , 'verify whether vehicle is following correctly or not'
* Refactoring and Delete needless parts
* Runtime Manager, increased frame height for Map tab viewing
* Runtime Manager, from Rviz to RViz
* Runtime Manager Simulation tab, delete Clock, Sim Time checkbox
* Runtime Manager Quick Start tab, fix launch_kill proc for Rviz
* Use tabs instead of spaces
  And remove needless debug print.
* Implement changing nice value in proc_manager
  Don't change uid because CAP_SYS_NICE does not permit to set smaller
  value.
* Runtime Manager Quick Start tab, Rviz remote
* Implement process manager
  Process manager provides
  - Set process cpu affinity
  - Set scheduling policy of process
  Process manager receives command from run time manager and change process
  configuration, and returns result to runtime manager.
* Add setup_tf.launch to set rosparams.
* Create lane_navi.launch
* Change to get rosparams as the arguments when booting
* Modify launch files to specify source camera
* Merge pull request `#166 <https://github.com/CPFL/Autoware/issues/166>`_ from CPFL/develop-multi-lane
  Develop multi lane
* Merge pull request `#161 <https://github.com/CPFL/Autoware/issues/161>`_ from CPFL/accelerate_euclidean_cluster
  Accelerate euclidean cluster
* Runtime Manager, add Autoware logo, update Icon
* ndt_matching supports setup tab.
* Add new parameters
* Make variable names more understandable
* Runtime Manager Setup tab, fix about Vehicle Model path
* Runtime Manager Setup tab, fix PCD Binarizer path
* Runtime Manager Setup tab, add Setup tab
* Runtime Manger Computing tab, add sched policy GUI
* Runtime Manager, update get_top() call to built-in next() call
* Create setting panel
* Runtime Manager Computing tab, update sys app link design
* Runtime Manager Map tab, delete comments
* Runtime Manager Map tab, add pcd tools
* Some Changes
* Change to use setting panel of lane_select
* Define new msgs in CMakelists
* Create new msgs
* added lack things
* created ConfigTwistFilter message
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Ros-parameterize all defined value in points2vscan
  Now we can specify parameters value from launch file
* Fix package name which has velodyne launch files
* Runtime Manager Map tab, fix pcd load OK msg at filed
* Merge pull request `#99 <https://github.com/CPFL/Autoware/issues/99>`_ from CPFL/driving-planner
  Update driving_planner and computing.yaml
* Runtime Manager, fix bottom area height
* Runtime Manager Computing tab, fix remain link at item hide
* Runtime Manager Computing tab, fix cmd_param drop
* Make projection matrix source selectable
  I modified nodes that subscribe /projection_matrix
  so that we can specify the topic name from launch file
* Make camera_info source selectable
  I modified nodes that subscribe /camera/camera_info
  so that we can specify the topic name from launch file
* KLT based Multi Tracking
  -Added Launch file access from RTM
  -Modified ImageViewer to show circles instead of rectangles
* Set topic name according to the number of connected camera
  Because calibration_publisher should publish each camera information
  when multiple cameras are connected to the system.
  In that case, calibration_publisher.launch must be executed with
  "name_space" argument that specifies corresponded camera.
* some change
* KLT based Multi Tracking
  -Added Launch file access from RTM
  -Modified ImageViewer to show circles instead of rectangles
* Show message box when calibration_path is none.
  Sensingタブの[config]でパスを設定していない場合、センサノードを起動しない
  でMessageBoxを出すようにしました。
* Merge branch 'master' into driving-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/driving_planner/nodes/lattice_trajectory_gen/lattice_trajectory_gen.cpp
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'master' into develop-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/waypoint_follower/CMakeLists.txt
* Runtime Manager Computing tab, modify ndt_matching config dialog
* Runtime Manager Computing tab, update ndt_matching config dialog
* Runtime Manager Computing tab, add children setting, modify default nice val
* Runtime Manager Computing tab, add sys link and cpu settings
* Merge pull request `#103 <https://github.com/CPFL/Autoware/issues/103>`_ from CPFL/add-number-of-zeros-behind-parameter
  Add number of zeros behind parameter
* Add number_of_zeros_behind parameter
* Merge pull request `#81 <https://github.com/CPFL/Autoware/issues/81>`_ from CPFL/rcnn
  Integration of RCNN object detection on Autoware
* Rename number_of_zeros parameter
* Remove unused message
* Runtime Manger, chaged icon, RTM to RUN
* Update driving_planner and computing.yaml
* parameterized
* renamed ConfigLaneFollower.msg to ConfigWaypointFollower.msg
* modified somethings in computing tab
* Change parameter order
* Merge branch 'master' of https://github.com/CPFL/Autoware into develop-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/driving_planner/nodes/velocity_set/velocity_set.cpp
  ros/src/util/packages/runtime_manager/msg/ConfigVelocitySet.msg
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* added velocity_source parameter on waypoint_saver
* Add new parameters
* As suggested by @syohex
  Thanks
* Added files for RCNN node
* Integration of RCNN object detection on Autoware
  **Added a new library librcnn, which executes the object recognition using the Caffe framework, specifically the fast-rcnn branch.
  git clone --recursive https://github.com/rbgirshick/fast-rcnn.git
  -Requires CUDA for GPU support.
  To take advantage of cuDNN, at least CUDA 7.0 and a GPU with 3.5 compute capability is required.
  -Compile Caffe, located in caffe-fast-rcnn.
  Complete the requisites:http://caffe.berkeleyvision.org/install_apt.html
  -Download the pretrained models:
  http://www.cs.berkeley.edu/~rbg/fast-rcnn-data/voc12_submission.tgz
  -Modify the CMakeFiles and point them to your caffe and models directories.
  **Modified KF to use the new NMS algorithm
  **Modified Range fusion, it will not execute unnecesary fusions.
  **Added Configuration Messages to Runtime manager and RCNN node launch files
* Runtime Manager, fix bottom top5 showing from thread
* Merge pull request `#63 <https://github.com/CPFL/Autoware/issues/63>`_ from CPFL/develop-sensor-gnss
  Develop sensor gnss
* Update sensing.yaml
  Added support for ros pgrey "native "driver
* Add new parameters for velocity_set
* Change package of velocity_set from waypoint_follower to driving_planner in runtime manager
* Runtime Manager, add top5 showing
* added simulation mode
* Runtime Manager Topics tab, fix topic echo off, immediately
* Runtime Manager Topics tab, Echo ckbox default OFF, syslog OFF
* Runtime Manager Statu tab, stdout,stderr default off
* Runtime Manager Status tab, modify top cmd view width
* added path_select
* Runtime Manager Status tab, update top command results font
* modified sensing.yaml
* Merge branch 'master' into develop-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/driving_planner/nodes/velocity_set/velocity_set.cpp
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Runtime Manager, brushup about link color setting
* Runtime Manager, brushup about wx.BoxSizer
* Runtime Manager Status tab, que clear at Stdout OFF and Stderr OFF
* Runtime Manager Sensing Tab, add config to Javad
* Merge pull request `#56 <https://github.com/CPFL/Autoware/issues/56>`_ from CPFL/mod_dpm_ocv
  Mod dpm ocv
* Add velocity_set dialog to Runtime Manager Computing tab
* Add ConfigVelocitySet.msg
* added twist filter node
* Runtime Manager, update about ndt_stat
* Show lane_stop configurations
* modified velocity_set
* Merge pull request `#29 <https://github.com/CPFL/Autoware/issues/29>`_ from CPFL/bug-fix
  fix typo
* fix typo
* Modified dpm_ocv so that making executing CPU, GPU, car detection and pedestrian detection selectable
* Runtime Manager, update red alert condition
* Moved dpm_ocv.launch from runtime_manager/scripts to cv_tracker/launch
* Add waypoint_clicker
* Added checkbox to runtime manger in order to boot baumer camera node
* Add ladybug node
* Runtime Manager, update load bar color
* Runtime Manager, add top cmd alert level setting
* Runtime Manager Sensing tab, add params to calibration publisher
* Runtime Manager Computing tab, add region_tlr diloag
* Runtime Manager Computing tab, add feat_proj dialog and adjst_xy msg
* Runtime Manager Computing tab, add feat_proj dialog and adjust_xy msg
* Runtime Manager, update top command setting
* Runtime Manager, add bar to system load info
* Add the state lattice motion planning features
* Initial commit for public release
* Contributors: AMC, Abraham, Abraham Monrroy, Hiroki Ohta, Manato Hirabayashi, Matthew O'Kelly, Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, W. Gustavo Cevallos, Yuki Kitsukawa, Yukihiro Saito, h_ohta, kondoh, kondoh2, niwasaki, pdsljp, syouji, yukikitsukawa
