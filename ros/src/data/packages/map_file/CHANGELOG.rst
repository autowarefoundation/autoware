^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package map_file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fixes for catkin_make
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
* Feature/points map filter (`#1658 <https://github.com/CPFL/Autoware/issues/1658>`_)
  * add points_map_filter node
  * add passthrough filter
  * fix filter function
  * apply clang-format
  * add README.md
* Contributors: Esteve Fernandez, Masaya Kataoka, amc-nu

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* [fix] PascalCase messages (`#1408 <https://github.com/CPFL/Autoware/issues/1408>`_)
  * Switch message files to pascal case
  * Switch message names to pascal case in Runtime Manager
  * Switch message names to pascal case in *.yaml
  * Rename brake_cmd and steer_cmd to BrakeCmd and SteerCmd in main.yaml
* Contributors: Esteve Fernandez

1.8.0 (2018-08-31)
------------------
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Esteve Fernandez, Kenji Funaoka

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* -Added support fot VMap colouring to Left Traffic signals (`#988 <https://github.com/CPFL/Autoware/pull/988>`_)
  -Added Lane number on tlr_superimpose
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
* use header.frame_id included in initialpose topic
* Initial modifications to feat_proj, tlr, context and vector_map loader, server and client to support different types of traffic signals
* Contributors: AMC, Yamato ANDO

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
* compilation issues
* added install targets
  changed finding pcl
  removed unneeded dependencies
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
* fix circular-dependency
* Contributors: Shohei Fujii

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
* Remove unnecessary error checks
* Delete map_file messages
* Refactoring
* Add error check for vector_map_loader
* Add download mode for vector_map_loader
* Have to be under_scored filename
* Rename wrong directory and filename
* Fix typos around map_db
* Add vector_map_loader corresponding to new road objects
* Add draft proposal of vector_map_loader
* Runtime Manager, update points_map_loader for SIGINT termination
* add const to errp read only parameter
* Runtime Manager Quick Start tab, fix Map load OK label
* Add module graph tool
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Rewrite points_map_loader
  Rewrite the entire main program.
  Delete a noisy debug message in library.
* Use pcd download thread too
  Existing callbacks use pcd download thread too.
* Add look ahead downloader
* Implement request queue to download pcd
* Are not DEBUG_PRINT
  These outputs are used by Runtime Manager.
* Move output of load message
* Fix handling of /pmap_stat
  Needn't buffer messages, should be lached.
  Add initialization code.
* Default variable is 1000 msec
* Fix update_rate
* Redesign map_downloader dialog
* Add ROS parameters for HTTP server
* Don't require initial position
* Delete file by the failure of download
  If libcurl fails to download, obtained file is deleted.
* Check HTTP response code
* Move std:ofstream::close
* Add digest access authentication
* Stop publishing messages of lane namespace
* Refactoring CMakeLists.txt
  Remove absolute paths by using cmake features and pkg-config.
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Merge map_db with map_file.
* Fix road sign warning on Rviz
* Initial commit for public release
* Contributors: Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, kondoh, syouji
