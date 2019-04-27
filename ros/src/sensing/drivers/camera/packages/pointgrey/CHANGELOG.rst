^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pointgrey_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* Include app name in log message
* To allow FPS to be set, it requires first to disable FrameRateAuto property
* fix framerate check
* Feature/spinnaker (`#1984 <https://github.com/CPFL/Autoware/issues/1984>`_)
  * add spinnaker driver
  * modified for multithreading
  * fix CMakeLists.txt
  * fix CMakeLists.txt
  * fix spinnaker launch file
  * delete unused files
  * fix maintainer
  * fix thread function
  * add license and fix default value
  * modify README
  * modify computing.yml
  * * Added install commands
  * Don't force raw image format
  * Removed mutex causing multiple cameras to don't publish
  * Removed crash due to the use of incompatible API on a camera
  * Switched to ROS logging instead of std out
  * Commented DLTL code
  * Removed namespace and pixel format
  * enable to change fps
  check device type and set DeviceLinkThroughputLimit
  * Check pointers' validity
  * applied clang format
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
* Contributors: Abraham Monrroy Cano, amc-nu, wep21

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
* [feature] Initial integration of Boson FLIR ADK camera driver for Autoware (`#1809 <https://github.com/CPFL/Autoware/issues/1809>`_)
  * Initial integration of Boson FLIR ADK camera driver for Autoware
  * Added README description for Boson FLIR ADK
* Fix Ros/ROS naming convention
* Contributors: Abraham Monrroy Cano, Esteve Fernandez

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------

1.8.0 (2018-08-31)
------------------
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* Added ID for multiple camera frames (`#1296 <https://github.com/CPFL/Autoware/pull/1296>`_)
  * Added ID for multiple camera frames
  * Fixed indexing for Ladybug camera
  * Fixed camera index on grasshopper
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Abraham Monrroy, Esteve Fernandez, Kenji Funaoka

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
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
* [feature] Add timeout to the grasshopper camera node. (`#1154 <https://github.com/CPFL/Autoware/pull/1154>`_)
  * Added timeout to the grasshopper camera node.
  * Added timeout to the launch file
* Autoware PointGrey driver package README (`#1145 <https://github.com/CPFL/Autoware/pull/1145>`_)
  * Autoware PointGrey driver package README
  * Pushed latest README
  * Edited README
* [feature] Grasshopper3 node modified to set the mode and pixel format (`#1105 <https://github.com/CPFL/Autoware/pull/1105>`_)
  * Grasshopper3 node modified to set the mode and pixel format
  * Baumer Type Readme
  * Baumer Auto Exposure Algorithm improvement
  * Added Documentation to the package
  * Added runtime manager param controls for both Ladybug and Grasshopper nodes
* Contributors: Abraham Monrroy, Kosuke Murakami

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
* Fixed mirrored images on Ladybug camera (`#906 <https://github.com/cpfl/autoware/issues/906>`_)
* Runtime manager updated to new package names (`#870 <https://github.com/cpfl/autoware/issues/870>`_)
  [fix] Runtime manager updated to new pgrey package names
* [Feature] Updates to Pointgrey package/Ladybug node (`#852 <https://github.com/cpfl/autoware/issues/852>`_)
  * Added support for FindXerces on Indigo (and Cmake less than 3.1.3)
  Changed default scaling value to 20% of original image size (to improve performance)
  Changed published image format from RGB to Bayer(to reduce bag size)
  * Tested on real camera, updated
* Contributors: Abraham Monrroy, Yamato ANDO

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
* add web ui
* Fix cmake and package
* Fix build error
* Contributors: Akihito OHSATO, Yusuke FUJII, hironari.yashiro

1.4.0 (2017-08-04)
------------------
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

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
* Fix ladybug driver
* Add module graph tool
* Fix typo
* Add encoding check to use bayer image format
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Add ladybug node
* Initial commit for public release
* Contributors: Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi
