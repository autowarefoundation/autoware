^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_processor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
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
* Fix license notice in corresponding package.xml
* Contributors: Abraham Monrroy Cano, amc-nu

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
* Fix Ros/ROS naming convention
* Contributors: Esteve Fernandez

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* [fix] Added option to publish to specific camera frame on camera publisher (`#1565 <https://github.com/CPFL/Autoware/issues/1565>`_)
  * * Added option to publish to specific camera frame on camera publisher
  * fixes to the node
  * Added New line to UI on each param.
  * * Updates to launch files using calibration publisher
  * Updated naming after develop merge
  * Updated suscription type to topic for calibration publisher
* Contributors: Abraham Monrroy Cano

1.8.0 (2018-08-31)
------------------
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Esteve Fernandez, Kenji Funaoka

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
* Removed flawed subtree
* Contributors: AMC, Abraham Monrroy, Kosuke Murakami

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
* Initial release of image processor (`#908 <https://github.com/cpfl/autoware/issues/908>`_)
  -rotation
  -rectification
  nodes included
* Contributors: Abraham Monrroy

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
