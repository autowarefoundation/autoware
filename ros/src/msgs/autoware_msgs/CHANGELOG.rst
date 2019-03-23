^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* [Feature] Rebuild decision maker (`#1609 <https://github.com/CPFL/Autoware/issues/1609>`_)
* Fix license notice in corresponding package.xml
* Cleanup WaypointState.msg and add sttering_state=STR_BACK (`#1822 <https://github.com/CPFL/Autoware/issues/1822>`_)
* Initial release of object filter
* Contributors: Abraham Monrroy, Kenji Funaoka, amc-nu, s-azumi

1.10.0 (2019-01-17)
-------------------
* Feature/perception visualization cleanup (`#1648 <https://github.com/CPFL/Autoware/issues/1648>`_)
  * * Initial commit for visualization package
  * Removal of all visualization messages from perception nodes
  * Visualization dependency removal
  * Launch file modification
  * * Fixes to visualization
  * Error on Clustering CPU
  * Reduce verbosity on markers
  * intial commit
  * * Changed to 2 spaces indentation
  * Added README
  * Fixed README messages type
  * 2 space indenting
  * ros clang format
  * Publish acceleration and velocity from ukf tracker
  * Remove hardcoded path
  * Updated README
  * updated prototype
  * Prototype update for header and usage
  * Removed unknown label from being reported
  * Updated publishing orientation to match develop
  * * Published all the trackers
  * Added valid field for visualization and future compatibility with ADAS ROI filtering
  * Add simple functions
  * Refacor code
  * * Reversed back UKF node to develop
  * Formatted speed
  * Refactor codes
  * Refactor codes
  * Refactor codes
  * Refacor codes
  * Make tracking visualization work
  * Relay class info in tracker node
  * Remove dependency to jskbbox and rosmarker in ukf tracker
  * apply rosclang to ukf tracker
  * Refactor codes
  * Refactor codes
  * add comment
  * refactor codes
  * Revert "Refactor codes"
  This reverts commit 135aaac46e49cb18d9b76611576747efab3caf9c.
  * Revert "apply rosclang to ukf tracker"
  This reverts commit 4f8d1cb5c8263a491f92ae5321e5080cb34b7b9c.
  * Revert "Remove dependency to jskbbox and rosmarker in ukf tracker"
  This reverts commit 4fa1dd40ba58065f7afacc5e478001078925b27d.
  * Revert "Relay class info in tracker node"
  This reverts commit 1637baac44c8d3d414cc069f3af12a79770439ae.
  * delete dependency to jsk and remove pointcloud_frame
  * get direction nis
  * set velocity_reliable true in tracker node
  * Add divided function
  * add function
  * Sanity checks
  * Relay all the data from input DetectedObject
  * Divided function work both for immukf and sukf
  * Add comment
  * Refactor codes
  * Pass immukf test
  * make direction assisted tracking work
  * Visualization fixes
  * Refacor codes
  * Refactor codes
  * Refactor codes
  * refactor codes
  * refactor codes
  * Refactor codes
  * refactor codes
  * Tracker Merging step added
  * Added launch file support for merging phase
  * lane assisted with sukf
  * Refactor codes
  * Refactor codes
  * * change only static objects
  * keep label of the oldest tracker
  * Static Object discrimination
  * Non rotating bouding box
  * no disappear if detector works
  * Modify removeRedundant a bit
  * Replacement of JSK visualization for RViz Native Markers
  * Added Models namespace to visualization
  * Naming change for matching the perception component graph
  * * Added 3D Models for different classes in visualization
  * 2D Rect node visualize_rects added to visualization_package
* Fix Ndt/NDT naming convention
* Contributors: Abraham Monrroy Cano, Esteve Fernandez

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* Moved CAN mesages to autoware_can_msgs
* Moved configuration messages to autoware_config_msgs
* [fix] PascalCase messages (`#1408 <https://github.com/CPFL/Autoware/issues/1408>`_)
  * Switch message files to pascal case
  * Switch message names to pascal case in Runtime Manager
  * Switch message names to pascal case in *.yaml
  * Rename brake_cmd and steer_cmd to BrakeCmd and SteerCmd in main.yaml
* Feature/compare map filter (`#1559 <https://github.com/CPFL/Autoware/issues/1559>`_)
  * add compare map filter
  * add README
  * add copyright
  * change default parameter
  * fix typo
  * clang-format
  * Revert "clang-format"
  This reverts commit 95869328f35f6ed1e918c26901ad36ab9737e466.
  * retry clang-format
* Contributors: Esteve Fernandez, YamatoAndo

1.8.0 (2018-08-31)
------------------
* Support old behavior of insert static object for obstacle avoidance testing
  Only one simulated car available in the runtime manager
  update for copywrite note
  insert autoware_build_flags to new nodes
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
* add max_scan_range to ConfigNDTMapping/ConfigApproximateNDTMapping
* Update OpenPlanner libraries (op_planner, op_utitity, op_ros_helpers)
  Update ring ground filter with latest implementation
  Update lidar_kf_contour_track with latest implementation
  Add op_utilities nodes (op_bag_player, op_data_logger, op_pose2tf)
  Modify autoware_msgs for OpenPlanner use (CloudCluster, DetectedObject, lane, waypoint)
  Update UI computing.yaml for the new nodes and modifies parameters
  Update UI sensing.yaml for updated ring_ground_filter params
* Modify points_concat_filter to support up to 8 lidars
* Contributors: Abraham Monrroy, Akihito Ohsato, hatem-darweesh, yukikitsukawa

1.7.0 (2018-05-18)
------------------
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
* Rename class and functions filter->replan
* Fix message
* Fix config message path
* change can_translator
  - Support to vehicle_status(can intermediate layer)
  - Separate the can translator and the odometry.
  - Support to output vehicle autonomous mode
* add vehicle_status msg
* Add end point offset option
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
* add ctrl_cmd/cmd/linear_acceletion
* Correspond to new version of waypoint_csv(for decision_maker)
* fix runtime_manager layout and description
* Add config_callback for online waypoint tuning
* Separate configration for speed planning against obstacle/stopline (Note: no logics changed)
* parametrize detection area
* add ratio for stopline target
* Add a transition to stopstate to re-start only manually
* add new param for decision_maker
* Contributors: Abraham Monrroy, Akihito Ohsato, Dejan Pangercic, Kosuke Murakami, Yamato ANDO, Yuma, Yuma Nihei, Yusuke FUJII

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* Contributors: Yusuke FUJII

1.6.0 (2017-12-11)
------------------
* Prepare release for 1.6.0
* Added support to publish result of multiple traffic signals according to the lane
  VectorMapServer Support to publish signals on current lane if current_pose and final_waypoints available
* Initial modifications to feat_proj, tlr, context and vector_map loader, server and client to support different types of traffic signals
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
* Support to lanechange similar to state_machine(old) package
* Changed path state recognition to the way based on /lane_waypoints_array
* Fix build error, add msg definition
* Rename and merge msgs
* add path velocity smoothing
* add msg of waypointstate for decision_maker
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
* refactor code
* refactor code
* refactor msg and add blinker to msg
* Add ground_filter config for runtime_manager (`#828 <https://github.com/cpfl/autoware/issues/828>`_)
* Ray Ground Filter Initial Commit
* add approximate_ndt_mapping (`#811 <https://github.com/cpfl/autoware/issues/811>`_)
* add new msg and rename msg
* add mqtt sender
* Contributors: AMC, Akihito Ohsato, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, hatem-darweesh

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
* Add decision_maker config
* add config parameter
* autoware_msgs does not depend on jsk_rviz_plugin, cmake and package.xml were not correct
* Contributors: Dejan Pangercic, Yusuke FUJII

1.4.0 (2017-08-04)
------------------
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
