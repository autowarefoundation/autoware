^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package points_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* Revert "Fix/health checker (`#2012 <https://github.com/CPFL/Autoware/issues/2012>`_)" (`#2037 <https://github.com/CPFL/Autoware/issues/2037>`_)
  This reverts commit e4187a7138eb90ad6f119eb35f824b16465aefda.
  Reverts `#2012 <https://github.com/CPFL/Autoware/issues/2012>`_
  Merged without adequate description of the bug or fixes made
* Fix/health checker (`#2012 <https://github.com/CPFL/Autoware/issues/2012>`_)
* [fix] Install commands for all the packages (`#1861 <https://github.com/CPFL/Autoware/issues/1861>`_)
  * Initial fixes to detection, sensing, semantics and utils
  * fixing wrong filename on install command
  * Fixes to install commands
  * Hokuyo fix name
  * Fix obj db
  * Obj db include fixes
  * End of final cleaning sweep
  * Incorrect command order in runtime manager
  * Param tempfile not required by runtime_manager
  * * Fixes to runtime manager install commands
  * Remove devel directory from catkin, if any
  * Updated launch files for robosense
  * Updated robosense
  * Fix/add missing install (`#1977 <https://github.com/CPFL/Autoware/issues/1977>`_)
  * Added launch install to lidar_kf_contour_track
  * Added install to op_global_planner
  * Added install to way_planner
  * Added install to op_local_planner
  * Added install to op_simulation_package
  * Added install to op_utilities
  * Added install to sync
  * * Improved installation script for pointgrey packages
  * Fixed nodelet error for gmsl cameras
  * USe install space in catkin as well
  * add install to catkin
  * Fix install directives (`#1990 <https://github.com/CPFL/Autoware/issues/1990>`_)
  * Fixed installation path
  * Fixed params installation path
  * Fixed cfg installation path
  * Delete cache on colcon_release
* Feature/autoware health checker (`#1943 <https://github.com/CPFL/Autoware/issues/1943>`_)
* Fix license notice in corresponding package.xml
* Contributors: Abraham Monrroy Cano, Geoffrey Biggs, Masaya Kataoka, amc-nu

1.10.0 (2019-01-17)
-------------------
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
* Contributors: Esteve Fernandez

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* Moved configuration messages to autoware_config_msgs
* renamed topics to match, sensing workflow (`#1600 <https://github.com/CPFL/Autoware/issues/1600>`_)
  [fix] renamed topics to match, sensing workflow on CompareMapFilter
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
* Contributors: Abraham Monrroy, Esteve Fernandez, YamatoAndo

1.8.0 (2018-08-31)
------------------
* Fix Indigo build issues
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* Update OpenPlanner libraries (op_planner, op_utitity, op_ros_helpers)
  Update ring ground filter with latest implementation
  Update lidar_kf_contour_track with latest implementation
  Add op_utilities nodes (op_bag_player, op_data_logger, op_pose2tf)
  Modify autoware_msgs for OpenPlanner use (CloudCluster, DetectedObject, lane, waypoint)
  Update UI computing.yaml for the new nodes and modifies parameters
  Update UI sensing.yaml for updated ring_ground_filter params
* Replaced yaml-cpp library flag
* Fix finding yaml-cpp by pkg_check_modules
* Add -lyaml-cpp
* Remove yaml-cpp find package
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Fix cmake and remove msg from runtime manager
* Fix assertion condition
* fix review comments
  - fix CMakeFiles.txt and package.xml related yaml-cpp
  - use input_topics_size\_
  - add brackets
* apply clang-format
* Modify runtime_manager
* apply clang-format
* Modify points_concat_filter to support up to 8 lidars
* Contributors: Akihito Ohsato, Esteve Fernandez, Kenji Funaoka, Yusuke FUJII, hatem-darweesh

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* [fix] Fixes for all packages and dependencies (`#1240 <https://github.com/CPFL/Autoware/pull/1240>`_)
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
  * More in detail dependencies fixes for more packages
  * GLEW library for ORB
  * Ignore OrbLocalizer
  * Ignore Version checker
  * Fix for driveworks interface
  * driveworks not catkinpackagedd
  * Missing catkin for driveworks
  * libdpm opencv not catkin packaged
  * catkin lib gnss  not included in obj_db
  * Points2Polygon fix
  * More missing dependencies
  * image viewer not packaged
  * Fixed SSH2 detection, added viewers for all distros
  * Fix gnss localizer incorrect dependency config
  * Fixes to multiple packages dependencies
  * gnss plib and package
  * More fixes to gnss
  * gnss dependencies for gnss_loclaizer
  * Missing gnss dependency for gnss on localizer
  * More fixes for dependencies
  Replaced gnss for autoware_gnss_library
  * gnss more fixes
  * fixes to more dependencies
  * header dependency
  * Debug message
  * more debug messages changed back to gnss
  * debud messages
  * gnss test
  * gnss install command
  * Several fixes for OpenPlanner and its lbiraries
  * Fixes to ROSInterface
  * More fixes to robotsdk and rosinterface
  * robotsdk calibration fix
  * Fixes to rosinterface robotsdk libraries and its nodes
  * Fixes to Qt5 missing dependencies in robotsdk
  * glviewer missing dependencies
  * Missing qt specific config cmake for robotsdk
  * disable cv_tracker
  * Fix to open planner un needed dependendecies
  * Fixes for libraries indecision maker
  * Fixes to libraries decision_maker installation
  * Gazebo on Kinetic
  * Added Missing library
  * * Removed Gazebo and synchonization packages
  * Renames vmap in lane_planner
  * Added installation commands for missing pakcages
  * Fixes to lane_planner
  * Added NDT TKU Glut extra dependencies
  * ndt localizer/lib fast pcl fixes
  re enable cv_tracker
  * Fix kf_lib
  * Keep industrial_ci
  * Fixes for dpm library
  * Fusion lib fixed
  * dpm and fusion header should match exported project name
  * Fixes to dpm_ocv  ndt_localizer and pcl_omp
  * no fast_pcl anymore
  * fixes to libdpm and its package
  * CI test
  * test with native travis ci
  * missing update for apt
  * Fixes to pcl_omp installation and headers
  * Final fixes for tests, modified README
  * * Fixes to README
  * Enable industrial_ci
  * re enable native travis tests
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
* Editorial changes to README files. See `#1124 <https://github.com/CPFL/Autoware/pull/1124>`_. (`#1125 <https://github.com/CPFL/Autoware/pull/1125>`_)
* Contributors: Abraham Monrroy, David, Kosuke Murakami

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
* decoupled ray ground filter into lib and exe, added unit test (`#932 <https://github.com/cpfl/autoware/issues/932>`_)
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* Feature/ring_ground_filter parameter description (`#884 <https://github.com/cpfl/autoware/issues/884>`_)
  * Added a README file for ground_filter tuning
  * Moved and formatted Patipon instructions on ring_ground_filter
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
* Solved conflicts by ring filter config message naming change
* Add ground_filter config for runtime_manager (`#828 <https://github.com/cpfl/autoware/issues/828>`_)
* Added Compilation fix for Kinect
* Added descriptions to the params in launch file
* Ray Ground Filter Initial Commit
* Contributors: AMC, Abraham Monrroy, Akihito Ohsato, Yamato ANDO, christopherho-ApexAI, hatem-darweesh

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
* [hotfix] fixes to lidar_tracker package(`#787 <https://github.com/cpfl/autoware/issues/787>`_)
  -Fixed a typo in the ground_filter launch file from points_preprocessor
  -Fixed ID duplication in kf_lidar_tracker
  Tested on Ubuntu 14.04 and 16.04
* Contributors: Abraham Monrroy, Yusuke FUJII

1.4.0 (2017-08-04)
------------------
* link to documentation
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Update ground_filter.launch
* Update ground_filter.launch
  Added params to launch file
* Typo Fix
* Fixed a bug that caused missing points
* Fixed linking error on 16.04
* Modified as suggested by @dejanpan on `#655 <https://github.com/cpfl/autoware/issues/655>`_
* -Standarized code
  -Added support for the 3 Velodyne Sensors models (use model_sensor {16,32,64})
  -Parametrized
* Test adding interface
* Try to add interface
* New version of ground_filter
* Contributors: AMC, Abraham Monrroy, Dejan Pangercic, Patiphon Narksri

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
* Rename package name.
  data_filter -> filters
  points_filter -> points_downsample
* Contributors: yukikitsukawa
