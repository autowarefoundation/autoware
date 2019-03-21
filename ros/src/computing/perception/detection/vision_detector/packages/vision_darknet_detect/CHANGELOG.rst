^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vision_yolo3_detect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* Removing CUDA dependencies for Darknet Yolov3 (`#1784 <https://github.com/CPFL/Autoware/issues/1784>`_)
  * Removing CUDA dependencies for Darknet yolov3
  If the host machine does not have CUDA, this will build the vision_darknet_detect package based on a pre-built darknet directory (which doesn't require CUDA as there are no CUDA dependencies for yolov3).
  * Update ros/src/computing/perception/detection/vision_detector/packages/vision_darknet_detect/CMakeLists.txt
  Co-Authored-By: K1504296 <greytrt@gmail.com>
* Fix license notice in corresponding package.xml
* Initial release of object filter
* Contributors: Abraham Monrroy, Theodore, amc-nu

1.10.0 (2019-01-17)
-------------------
* Fixes for catkin_make
* [fix] SSD detector, cmake colcon (`#1837 <https://github.com/CPFL/Autoware/issues/1837>`_)
  * Fixes for new colcon script on ssd cuda based node
  * Fixed to RTM and darknet launch files
  * catkin_fix
  * * catkin & colcon build successfully
  * reverted back run to devel space (for the time being)
* Switch to Apache 2 license (develop branch) (`#1741 <https://github.com/CPFL/Autoware/issues/1741>`_)
  * Switch to Apache 2
  * Replace BSD-3 license header with Apache 2 and reassign copyright to the
  Autoware Foundation.
  * Update license on Python files
  * Update copyright years
  * Add #ifndef/define _POINTS_IMAGE_H\_
  * Updated license comment
* Use colcon as the build tool (`#1704 <https://github.com/CPFL/Autoware/issues/1704>`_)
  * Switch to colcon as the build tool instead of catkin
  * Added cmake-target
  * Added note about the second colcon call
  * Added warning about catkin* scripts being deprecated
  * Fix COLCON_OPTS
  * Added install targets
  * Update Docker image tags
  * Message packages fixes
  * Fix missing dependency
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
* Fix Ros/ROS naming convention
* Fix Ssd/SSD naming convention
* Contributors: Abraham Monrroy Cano, Esteve Fernandez, amc-nu

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* Fix compile error (vision_darknet_detect.h:52:37: fatal error: autoware_msgs/ConfigSsd.h: No such file or directory)
* Moved configuration messages to autoware_config_msgs
* include fstream header (`#1608 <https://github.com/CPFL/Autoware/issues/1608>`_)
* Added support for custom class "names files" in darknet format. (`#1535 <https://github.com/CPFL/Autoware/issues/1535>`_)
  * Added support for custom class "names files" in darknet format.
  * Fixed launch file, not including source topic arg
  * Fix the default path of coco.names (`#1550 <https://github.com/CPFL/Autoware/issues/1550>`_)
* fixes two typos in yolo class name/id file (`#1484 <https://github.com/CPFL/Autoware/issues/1484>`_)
* Contributors: Abraham Monrroy, Esteve Fernandez, Jacob Lambert, Kenji Funaoka

1.8.0 (2018-08-31)
------------------
* fixes two typos in yolo class name/id file (`#1486 <https://github.com/CPFL/Autoware/pull/1486>`_)
* [Fix] README.md of vision_darknet_detect (`#1437 <https://github.com/CPFL/Autoware/pull/1437>`_)
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
* Contributors: Abraham Monrroy, Kenji Funaoka

1.7.0 (2018-05-16)
------------------
* Add  code in cmakelists
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* Remove history of sub-branches
* Add automatically-generated CHANGELOG.rst
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
* Contributors: Kenji Funaoka, Kosuke Murakami

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

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
