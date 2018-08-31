^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package runtime_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* Support old behavior of insert static object for obstacle avoidance testing
  Only one simulated car available in the runtime manager
  update for copywrite note
  insert autoware_build_flags to new nodes
* Add variable number of params for clustering_ranges and clustering_distances
* load multiple thres params from string: less rosparam
* Naming change from segment to clustering
* Add optional multiple threshold euclidean clustert
* Fix some comments and arguments
* bug fix in computing yaml
* Add launcher and parameters to runtime_manager
* Add only one segment distance
* Feature/std perception msg (`#1418 <https://github.com/CPFL/Autoware/pull/1418>`_)
  * New standard message definition for the perception nodes
  * New Detected Object message applied to:
  * SSD
  * Integrated RVIZ viewer
  * External Viewer
  * modified yolo2 and yolo3, compiles but cuda issues, trying different PC
  * Boiler plate for range vision fusion node
  * Added GenColors for Kinetic
  Typo fixes for yolo2
  * testing colors in Yolo3
  * Completed transformation, projection of 3D boxes
  * Fixed error on negative assignation
  * code clean up
  * removed yolo2 and yolo3, replaced by single darknet node. GUI launches yolo3 for now, to change. Pushing to test code on other PC.
  * Readme updated, added gitignore for data folder.
  * *Added Runtime manager UI for yolo2, yolo3.
  *Support tested for TinyYolo v2 and v3
  * Fusion Vision Range
  Icons for viewer
  * Range Vision Fusion node
  * Indigo cv im read
  * Indigo compiation fix
  * Topic renaming according to new spec
  * Try to fix arm64 stuff
  * * Added launch file
  * Added Runtime manager entry
  * * Added Publication of non fused objects
  * Fixed topic names
* widen app dialog for approximate_ndt_mapping
* add max_scan_range to ConfigNdtMapping/ConfigApproximateNdtMapping
* Modify Map loading for OpenPlanner, now it reads from Autoware vector map messages, old behavior still works but from launch file only.
  Delete way_planner, dp_planner from UI, but they still accessible from roslaunch.
* Fix Vector Map parser problem, tested with three different maps
  Fix Global Planning function for the new map modification
  Add OpenPlanner Simulator for perception, traffic lights, cars
  Add OpenPlanner new version to replace wp_planner and dp_planner
  Remove unnecessary files from OpenPlanner libraries
  Test Global and Local planning
  Test Tracking node (kf_contour_track)
  Test Simulation Nodes
  Test Utility Nodes
* Update op_utility files for csv files loading
  Update MappingHelpers with latest modifications
  Update PlanningHelpers with latest modifications
  add op_common_param node, for setting OpenPlanner parameter for all related nodes such as lidar_kf_contour_track
  Improve tracking by including size different in association function
  Update way_planner, dp_planner for compatibility with new Mapping Modifications, Map format is backward compatible
* Update OpenPlanner libraries (op_planner, op_utitity, op_ros_helpers)
  Update ring ground filter with latest implementation
  Update lidar_kf_contour_track with latest implementation
  Add op_utilities nodes (op_bag_player, op_data_logger, op_pose2tf)
  Modify autoware_msgs for OpenPlanner use (CloudCluster, DetectedObject, lane, waypoint)
  Update UI computing.yaml for the new nodes and modifies parameters
  Update UI sensing.yaml for updated ring_ground_filter params
* Renamed the package name to map_tf_generator and formatted the code with clang-format.
* Support Sick LD-MRS Lidar (`#1287 <https://github.com/CPFL/Autoware/pull/1287>`_)
  * renamed sick_driver to sick_lms5xx
  * Squashed 'ros/src/sensing/drivers/lidar/packages/sick/ldmrs/' content from commit 27976ff
  git-subtree-dir: ros/src/sensing/drivers/lidar/packages/sick/ldmrs
  git-subtree-split: 27976ff379263fdaab09e508bccdba7f9502be03
  * checkbox for Sick LD-MRS added
  * Added missing dependencies for sick_driver
* Fix cmake and remove msg from runtime manager
* I have created the tf_genarator package and changed it to use StaticTransformBroadcaster instead of call.
* Modify runtime_manager
* Modify points_concat_filter to support up to 8 lidars
* add tf generator `#1238 <https://github.com/CPFL/Autoware/pull/1238>`_
* Contributors: Abraham Monrroy, Akihito Ohsato, Azumi SUZUKI, Kosuke Murakami, Yuki Kitsukawa, hatem-darweesh, yukikitsukawa

1.7.0 (2018-05-18)
------------------
* Delete duplicated nodes on Runtime Manager
* Update runtime_manager/scripts/launch_files
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* [Fix] rename packages (`#1269 <https://github.com/CPFL/Autoware/pull/1269>`_)
  * rename lidar_tracker
  * Modify pf_lidar_track's cmake file
  * Refactor code
  * Rename from euclidean_lidar_tracker to lidar_euclidean_track
  * Rename from kf_contour_track to lidar_kf_contour_track
  * Rename from kf_lidar_track to lidar_kf_track, but need some modification in euclidean cluster(Cluster.h)
  * Rename from pf_lidar_tarck to lidar_pf_track
  * Rename range_fusion
  * Rename obj_reproj
  * Rename euclidean_cluster to lidar_euclidean_cluster_detect
  * Rename svm_lidar_detect to lidar_svm_detect
  * Rename kf_lidar_track to lidar_kf_track
  * Change version 1.6.3 to 1.7.0 in pacakge.xml
  * Modify CMake so that extrenal header would be loaded
  * Remove obj_reproj from cv_tracker
  * Add interface.yaml
  * Rename road_wizard to trafficlight_recognizer
  * create common directory
  * Add lidar_imm_ukf_pda_track
  * create vision_detector and moved cv
  * Modify interface.yaml and package.xml
  * remove dpm_ocv
  * moved directory
  * Delete unnecessary launch file
  * Delete rcnn related file and code
  * separated dummy_track from cv_tracker
  * separated klt_track from cv_tracker
  * Fix a cmake
  * Remove unnecessary dependency of lidar_euclidean_cluster_detect package
  * Rename image_segmenter to vision_segment_enet_detect
  * Remove unnecessary dependency of lidar_svm_detect package
  * separated kf_track and fix a some compiling issue
  * move viewers
  * merge ndt_localizer and icp_localizer, and rename to lidar_localizer
  * Remove unnecessary dependency of lidar_euclidean_track
  * moved image lib
  * add launch
  * lib move under lidar_tracker
  * Rename dpm_ttic to vision_dpm_ttic_detect
  * rename yolo3detector to vision_yolo3_detect
  * Modify cmake and package.xml in vision_dpm_ttic_detect
  * moved sourcefiles into nodes dir
  * moved sourcefiles into nodes dir
  * Move cv_tracker/data folder and delete cv_tracker/model folder
  * fix a package file and cmake
  * Rename yolo2 -> vision_yolo2_detect
  * fix a package file and cmake
  * Fix package name of launch file
  * Rename ssd to vision_ssd_detect
  * fixed cmake and package for decerese dependencies
  * remove top packages dir for detection
  * fixed cmake for cuda
  * Rename lane_detector to vision_lane_detect
  * Modify package.xml in lidar-related packages
  * Remove unnecessary dependencies in lidar_detector and lidar_tracker
  * Modify computing.yaml for dpm_ttic
  * Modify dpm_ttic launch file
  * Remove/Add dependencies to trafficlight_recognizer
  * Update data folder in dpm_ttic
  * Modified CMake and package file in dpm_ttic.
  * Remove src dir in imm_ukf_pda_track
  * removed unnecessary comments
  * rename lidar_tracker
  * Modify pf_lidar_track's cmake file
  * Refactor code
  * Rename from euclidean_lidar_tracker to lidar_euclidean_track
  * Rename from kf_contour_track to lidar_kf_contour_track
  * Rename from kf_lidar_track to lidar_kf_track, but need some modification in euclidean cluster(Cluster.h)
  * Rename from pf_lidar_tarck to lidar_pf_track
  * Rename range_fusion
  * Rename obj_reproj
  * Rename road_wizard to trafficlight_recognizer
  * Rename euclidean_cluster to lidar_euclidean_cluster_detect
  * Rename svm_lidar_detect to lidar_svm_detect
  * Rename kf_lidar_track to lidar_kf_track
  * Change version 1.6.3 to 1.7.0 in pacakge.xml
  * Modify CMake so that extrenal header would be loaded
  * Remove obj_reproj from cv_tracker
  * Add interface.yaml
  * create common directory
  * Add lidar_imm_ukf_pda_track
  * create vision_detector and moved cv
  * Modify interface.yaml and package.xml
  * remove dpm_ocv
  * moved directory
  * Delete unnecessary launch file
  * Delete rcnn related file and code
  * separated dummy_track from cv_tracker
  * separated klt_track from cv_tracker
  * Fix a cmake
  * Remove unnecessary dependency of lidar_euclidean_cluster_detect package
  * Rename image_segmenter to vision_segment_enet_detect
  * Remove unnecessary dependency of lidar_svm_detect package
  * separated kf_track and fix a some compiling issue
  * move viewers
  * merge ndt_localizer and icp_localizer, and rename to lidar_localizer
  * Remove unnecessary dependency of lidar_euclidean_track
  * moved image lib
  * add launch
  * lib move under lidar_tracker
  * Rename dpm_ttic to vision_dpm_ttic_detect
  * rename yolo3detector to vision_yolo3_detect
  * Modify cmake and package.xml in vision_dpm_ttic_detect
  * moved sourcefiles into nodes dir
  * moved sourcefiles into nodes dir
  * Move cv_tracker/data folder and delete cv_tracker/model folder
  * fix a package file and cmake
  * Rename yolo2 -> vision_yolo2_detect
  * fix a package file and cmake
  * Fix package name of launch file
  * Rename ssd to vision_ssd_detect
  * fixed cmake and package for decerese dependencies
  * remove top packages dir for detection
  * fixed cmake for cuda
  * Rename lane_detector to vision_lane_detect
  * Modify package.xml in lidar-related packages
  * Remove unnecessary dependencies in lidar_detector and lidar_tracker
  * Modify computing.yaml for dpm_ttic
  * Modify dpm_ttic launch file
  * Remove/Add dependencies to trafficlight_recognizer
  * Update data folder in dpm_ttic
  * Modified CMake and package file in dpm_ttic.
  * Remove src dir in imm_ukf_pda_track
  * Fix bug for not starting run time manager
  * Remove invalid dependency
* Return disable_decision_maker to rosparam
* Rename waypoint_replan to velocity_replanner
* Add tracking_frame
* Merge visualize_cloud_cluster.launch to imm_ukf_pda_tracker.launch
* Rename class and functions filter->replan
* Fix parameter limits and descriptions
* Refactor codes
* [feature] ndt matching safe monitor (`#1181 <https://github.com/CPFL/Autoware/pull/1181>`_)
  * Initial release of ndt_matching_monitor safety node
  * Added extra instruction
  * * Removed Rviz warnings
  * Added GNSS orientation if available
  * Added extra documentation
  * * Changed param name
  * Corrected README subtitle
  * * Added Topic for ndt status
  * Added published topic to readme
  * Integrated matching monitor into ndt_localizer
* Modify roslaunch params
* Add end point offset option
* Fix default filetring_mode parameter = false
* Fix/cmake cleanup (`#1156 <https://github.com/CPFL/Autoware/pull/1156>`_)
  * Initial Cleanup
  * fixed also for indigo
  * kf cjeck
  * Fix road wizard
  * Added travis ci
  * Trigger CI
  * Fixes to cv_tracker and lidar_tracker cmake
  * Fix kitti player dependencies
  * Removed unnecessary dependencies
  * messages fixing for can
  * Update build script travis
  * Travis Path
  * Travis Paths fix
  * Travis test
  * Eigen checks
  * removed unnecessary dependencies
  * Eigen Detection
  * Job number reduced
  * Eigen3 more fixes
  * More Eigen3
  * Even more Eigen
  * find package cmake modules included
  * More fixes to cmake modules
  * Removed non ros dependency
  * Enable industrial_ci for indidog and kinetic
  * Wrong install command
  * fix rviz_plugin install
  * FastVirtualScan fix
  * Fix Qt5 Fastvirtualscan
  * Fixed qt5 system dependencies for rosdep
  * NDT TKU Fix catkin not pacakged
  * Fixes from industrial_ci
* Add run-time manager script
* add gear p
* add ctrl_cmd/cmd/linear_acceletion
* Add new imm_ukf_pda_tracker feature
* [feature] Initial release of Yolo v3 node (`#1202 <https://github.com/CPFL/Autoware/pull/1202>`_)
  * Initial release of Yolo v3 node
  * Added extra documentation
  * * Missing header include
* fix description
* [Feature] region tlr mxnet (`#1048 <https://github.com/CPFL/Autoware/pull/1048>`_)
  * Initial commit of MxNet TLR based recognizer
  * Added result interpretation
  * Added
  -Score threshold,
  -latch-like trap to avoid sudden state changes,
  -latch threshold to set minimum number of instances before changing state
  * added mxnet to runtime manager
  * Fix the settings of runtime manager from apex version
  * Add launch file for region_tlr_mxnet
  * added path dialogue boxes to load model and params for mxnet
  * Add the settings of region_tlr_mxnet on runtime manager dialogue
* Correspond to new version of waypoint_csv(for decision_maker)
* Analog Devices ADIS16470 added to Runtime Manager
* fix runtime_manager layout and description
* Added support for Vimba SDK supportted cameras (`#1170 <https://github.com/CPFL/Autoware/pull/1170>`_)
* Add config_callback for online waypoint tuning
* Add velocity plan offset for system delay
* fix launch files for ros parameter
* [feature] Add timeout to the grasshopper camera node. (`#1154 <https://github.com/CPFL/Autoware/pull/1154>`_)
  * Added timeout to the grasshopper camera node.
  * Added timeout to the launch file
* Added RTM UI controls
* Add waypoint_filter functions
* add gmsl button to runtime manager
* [feature] Grasshopper3 node modified to set the mode and pixel format (`#1105 <https://github.com/CPFL/Autoware/pull/1105>`_)
  * Grasshopper3 node modified to set the mode and pixel format
  * Baumer Type Readme
  * Baumer Auto Exposure Algorithm improvement
  * Added Documentation to the package
  * Added runtime manager param controls for both Ladybug and Grasshopper nodes
* [update] Hokuyo 2D driver update (`#1091 <https://github.com/CPFL/Autoware/pull/1091>`_)
  * Replaced outdated Hokuyo node with new urg_node
  * Added description to RTM
  * Updated Hokuyo3D, added RTM UI for both 2D and 3D Hokuyo nodes
  * Requested changes attended
* Update sensing.yaml
  HDL-32C -> VLP-32C
* Added HDL32C to RTM
* Added hdl32c, fixed naming
* Added descriptions and optional calibration files. Defaults are used if not defined.
* improve config gui of NDT
* Feature/occupancygrid filter (`#1002 <https://github.com/CPFL/Autoware/pull/1002>`_)
  * Add grid map filter node
  * Add wayarea2grid node
  * Replace dist_transform with grid_map_filter
  * Add Runtime Manager UI for grid_map_filter node
  * Add UI for wayarea2grid node
  * Add instruction videos
  * Fix videos
  * Both node handles were private
  * Added Comments Documentation
  Code refactoring to follow standards
  Added libraries
  Separation of Vectormap Processing inside Clustering
  * Added documentation
  * Changed variable name
  * Added Road Occupancy Processor package
  * Added extra documentation
  Added commands to RunTimeManager
* add checkbox for IDS UI-3060CP (`#1058 <https://github.com/CPFL/Autoware/pull/1058>`_)
  * add checkbox for IDS UI-3060CP
  * Added description
* Add SICK LMS511 driver (`#1054 <https://github.com/CPFL/Autoware/pull/1054>`_)
  * Add SICK LMS511 driver
  * remove unnecessary dependencies
  * Name of package changed
  * Added Node Documentation
* Add deleting marker/buffers for replaying rosbag, and some fix
* Fix typo
* Separate speed vizualizer -> waypoint_marker/waypoint_velocity_vizualizer
* Modify window size of velocity_set app tab
* Separate configration for speed planning against obstacle/stopline (Note: no logics changed)
* parametrize detection area
* add ratio for stopline target
* add new param for decision_maker
* add new state button on runtime manager
* add new state for gui
* add points_topic in app dialog for points_downsampler so that we can select the input topic (`#990 <https://github.com/CPFL/Autoware/pull/990>`_)
* [Feature] Filter Clustered Objects using VectorMap info. (`#980 <https://github.com/CPFL/Autoware/pull/980>`_)
  * Switched from VectorMapServer to Image based filtering
  Added OpenMP support
  * Missing header
  * Added MapGrid configuration file
  Added parameterization to the occupancy grid size
* Fix unfinished merge branch 'develop'
* Add velocity visualizer with 3D graph.
* Contributors: AMC, Abraham Monrroy, Akihito Ohsato, Dejan Pangercic, Kenji Funaoka, Kosuke Murakami, TomohitoAndo, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yuma, Yuma Nihei, Yusuke FUJII, cirpue49, yukikitsukawa

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* add gmsl button to runtime manager
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* Contributors: Yusuke FUJII

1.6.0 (2017-12-11)
------------------
* Prepare release for 1.6.0
* add new state button on runtime manager
* add new state for gui
* Added support to Image Publisher for frames different than only "velodyne". (`#946 <https://github.com/cpfl/autoware/issues/946>`_)
* [fix] Image src correctly set in the config file of runtime manager for ssd node (`#939 <https://github.com/cpfl/autoware/issues/939>`_)
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
* - Add user interface option in runtime manage for local_planner and op_trajectory_generator
* fix computing.yaml
* add app dialog for fix2tfpose and nmea2tfpose (WIP)
* Fix feature/points2image bug multicam support (`#886 <https://github.com/cpfl/autoware/issues/886>`_)
  * pointgrey
  * Added New Calibration node
  * Added parameters, plane fitting
  * added mirror node, etc
  * Points2Image
  Calibration Publisher
  now works with multiple cameras using ros namespaces
  * Including only points2image
  * Added Launch file for points2 image specific for the ladybug camera
* [feature] Added NDT Mapping Descriptions and checkboxes to Runtime Manager UI (`#882 <https://github.com/cpfl/autoware/issues/882>`_)
  * Added Descriptions and checkboxes to Runtime Manager UI
  * Added also approximate ndt mapping descriptions
* remove unnecessary gui
* Add ndt_cpu library
* add path velocity smoothing
* add waypoint relay
* Runtime manager updated to new package names (`#870 <https://github.com/cpfl/autoware/issues/870>`_)
  [fix] Runtime manager updated to new pgrey package names
* Feature/fusion_filter - fusion multiple lidar (`#842 <https://github.com/cpfl/autoware/issues/842>`_)
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
* Feature/cloud transformer (`#860 <https://github.com/cpfl/autoware/issues/860>`_)
  * Added Cloud transformer node
  transforms pointcloud to a target frame
  * Added support for XYZIR point type
  * Added error checks when transformation unavailable
* add commonly buttons
* rename state tabs
* add select state tab
* moved images
* add use_gpu option for ndt_matching and ndt_mapping in app dialog
* Runtime Manager, fix wrong scrolling when checked on kinetic and hlink layout
* Runtime Manager, rosbag filename update when ROSBAG button on, and fix unnecessary ext.
* Add ground_filter config for runtime_manager (`#828 <https://github.com/cpfl/autoware/issues/828>`_)
* Added descriptions to the params in launch file
* Ray Ground Filter Initial Commit
* add approximate_ndt_mapping (`#811 <https://github.com/cpfl/autoware/issues/811>`_)
* fix bug and refactor code
* add new button for remote monitor to runtime_manager
* Contributors: AMC, Abraham Monrroy, Akihito Ohsato, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, anhnv-3991, hatem-darweesh, kondoh, yukikitsukawa

1.5.1 (2017-09-25)
------------------
* Release/1.5.1 (`#816 <https://github.com/cpfl/autoware/issues/816>`_)
  * fix a build error by gcc version
  * fix build error for older indigo version
  * update changelog for v1.5.1
  * 1.5.1
* Contributors: Yusuke FUJII

1.5.0 (2017-09-21)
------------------
* Update changelog
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
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Publish initial light color as green
* Change yaml file in runtime_manager
* Contributors: Dejan Pangercic, Patiphon Narksri, TomohitoAndo

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix a build issue due to autoware_msgs on the Indigo
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
* Added Video to Main Readme, added in Runtime Manager
* Yolo 2 Node completed
* add can2odom.launch
* Change OpenPlanner stand alone libraries names.
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
* add psutil 3.x to 1.x backward compatibility
* add measurement_range
  refactoring
* Runtime Manager, fix error at calibration_publisher button off
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
* Runtime Manager, add proc_wait_thread() for async kill proc
* Runtime Manager, add quit_select() and reload_computing_yaml()
* Kf Added
  Euclidean Cluster improved
* Fixes
* Added VectorMap Server support
* Lidar tracker restructuration
* Added output_frame param
  Allows to transform output coordinate frame of the bounding boxes and CloudClusterArray messages
* Added Ground removal as optional.
  Removed unused params
* Parametrization of Clustering params
* Contributors: AMC, Shohei Fujii, Yukihiro Saito, Yusuke FUJII, hatem-darweesh, kondoh, tange, yukikitsukawa

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Fix app display for vel_pose_connect
* size adjustment of config window
* add set_size_gdic() for adjust dialog size
* add max_iterations, min_scan_range and min_add_scan_shift
* add max_iterations
* Runtime Manager Map tab, add Point Type to PCD Filter and PCD Binarizer
* Add euclidean lidar track
* Add dummy tracking node
* Add autoware_connector instead of vel_pose_mux
* Fix computing.yaml
* Change Parameter range and initial value
* parameter from runtime manager
* Update sensing.yaml
* Add param bar of twist filter node in runtime manager
* add Gazebo button on Simulation tab
* Use integer size with temporal waypoints
* Add state_machine on runtime manager
* New simulator with angle and position errors
* Change minimun and maximum value of parameters of pure pursuit
* Change the default parameter
* Bug fix for linear interpolation flag and command velocity
* Add potential field in runtime manager
* Add topic publisher for steering robot
* Change parameter name in app tab
* Rewrite to change local planning to global planning
* Edit computing.yaml for lane_select and fix typo
* Add support for multi lane files
* Contributors: TomohitoAndo, Yukihiro Saito, h_ohta, kondoh, pdsljp, yukikitsukawa

1.0.1 (2017-01-14)
------------------
* Fix camera_ids.
* Contributors: USUDA Hisashi

1.0.0 (2016-12-22)
------------------
* Add dialog in Runtime Manager
* Runtime Manager Computing tab, fix euclidean_cluster default data
* Modify to launch the node in Runtime Manager
* add checkbox value of lane_stop parameter to pdic
* Added param to ignore points closer than a threshold
* Fix for using the twist of MKZ as current velocity
* Create red and green lanes from waypoint_saver CSV
* Node Name Change for cv tracker
* Added SSD node to CV Tracker
* Rename variables.
* Lidar segmentation (`#499 <https://github.com/cpfl/autoware/issues/499>`_)
  * Lidar tracker restructuration
  * Added points_preprocessor package, including; ground filtering and space filtering.
* Separate motion planning package
* Add get height
  If 'Get Height' checkbox is enabled on ndt_matching, you get height of initial pose by 2D Pose Estimate.
  This is height of nearest point in /points_map.
* Added output_frame param
  Allows to transform output coordinate frame of the bounding boxes and CloudClusterArray messages
* Lidar segmentation (`#486 <https://github.com/cpfl/autoware/issues/486>`_)
  Added CloudCluster and CloudClusterArray Message
  Clusters and its feats can now be accessed from outside the clustering node.
  Refer to the messages definition
* Update the check.launch for the tablet_socket because the node ndt_pcl change to ndt_matching
* Add parameter for subscribing topic
* Lidar segmentation (`#482 <https://github.com/cpfl/autoware/issues/482>`_)
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
* Runtime Manager Map tab, update to more simple implements
* Runtime Manager Computing tab, add use_crosswalk_detection to velocity_set
* Remove unused parameters
* Change variable name in ConfigWaypointFollower, calculate function for lookahead distance
* Runtime Manager, changed wx.Color to wx.Colour for wxPython3
* Runtime Manager, fix rosbag record stop
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
* add obj_enables utils for widget enable/disable
* refactoring config_dic search, add param sys
* Runtime Manager, add utils for dict and list
* Add lazy_ndt_mapping.
  Add checkbox for lazy_ndt_mapping in Computing tab.
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
* Runtime Manager, fix dialog showing after closeing by close box
* Removed *.orig files
* Change a subscribing topic  in points2image and vscan2points when synchronization is enabled
* Runtime Manager Setup tab, move rosparam tf_xxx setting from setup_tf.launch
* Runtime Manager Setup tab, add enable/disable toggle to radio box
* Runtime Manager Setup tab, add localizer radio box
* Change initial value
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
* merged master `#123 <https://github.com/cpfl/autoware/issues/123>`_, for lost cmd args
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
* Runtime Manager Computing tab, modify ndt_matching config dialog
* Runtime Manager Computing tab, update ndt_matching config dialog
* Runtime Manager Computing tab, add children setting, modify default nice val
* Runtime Manager Computing tab, add sys link and cpu settings
* Add number_of_zeros_behind parameter
* Rename number_of_zeros parameter
* Remove unused message
* Runtime Manger, chaged icon, RTM to RUN
* Update driving_planner and computing.yaml
* parameterized
* renamed ConfigLaneFollower.msg to ConfigWaypointFollower.msg
* modified somethings in computing tab
* Change parameter order
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
* Runtime Manager, brushup about link color setting
* Runtime Manager, brushup about wx.BoxSizer
* Runtime Manager Status tab, que clear at Stdout OFF and Stderr OFF
* Runtime Manager Sensing Tab, add config to Javad
* Add velocity_set dialog to Runtime Manager Computing tab
* Add ConfigVelocitySet.msg
* added twist filter node
* Runtime Manager, update about ndt_stat
* Show lane_stop configurations
* modified velocity_set
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
* Contributors: AMC, Abraham, Abraham Monrroy, Hiroki Ohta, Manato Hirabayashi, Matthew O'Kelly, Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, W. Gustavo Cevallos, Yukihiro Saito, h_ohta, kondoh, niwasaki, pdsljp, syouji, yukikitsukawa
