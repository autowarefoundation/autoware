^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package adi_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------

1.10.0 (2019-01-17)
-------------------

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------

1.8.0 (2018-08-31)
------------------
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Kenji Funaoka

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* Modify package xml version other than 1.6.3
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
* Added README on how to use subtree
* adi_driver subtree added
* Contributors: Abraham Monrroy, Kosuke Murakami, amc-nu, yukikitsukawa

1.0.1 (2018-02-02)
------------------
* fix deb path (`#8 <https://github.com/tork-a/adi_driver/issues/8>`_)
* workaround for run_tests on installed space (`#7 <https://github.com/tork-a/adi_driver/issues/7>`_)
  * enable deb build
  * add rosdoc.yaml
  * add .github_release.sh
  * Change photo of the sensor
  * Add author to package.xml (`#5 <https://github.com/tork-a/adi_driver/issues/5>`_)
  * Add urdf to install (`#5 <https://github.com/tork-a/adi_driver/issues/5>`_)
  * add roslaunch-check with_rviz:=true with_plot:=true
  * install test directory
  * workaround for run_tests on installed space
* Add adxl345 descrption into README.md (`#4 <https://github.com/tork-a/adi_driver/issues/4>`_)
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

1.0.0 (2018-01-21)
------------------
* note that you need to restart after addgroup (`#3 <https://github.com/tork-a/adi_driver/issues/3>`_)
* Add publish_tf argument for launch file
* Change to load model only if use rviz
* Update index.rst
* Fix schematics, add documents
  - Schematics of ADIS16470 cable was wrong
  - Add ADXL345 cable schematics
* Add docbuild (`#2 <https://github.com/tork-a/adi_driver/issues/2>`_)
  * add circle.yml
  * add docbuild command to CMakeLists.txt
  * Update index.rst contents
  * Put travis badge.
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534, Y. Suzuki

0.0.1 (2017-12-14)
------------------
* Add doc/index.rst
* Fix build errors
* Remove all error of roslint
* Add roslint settings
* Adjust header inclusion
* Add loop rate parameter
* Refactor adis16470 code
* Add and change copyrights
* Change copyright representation
* Change test more practical
* add .travis.yml (`#1 <https://github.com/7675t/adi_driver/issues/1>`)
* add xacro to pakcage.xml
* add rviz, imu_filter_madgwick to pakcage.xml
* fix layout in package.xml
* add rqt_plot to package.xml
* add .travis.yml, using generate_prerelease_script.py
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534
