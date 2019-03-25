^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package freespace_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* [Feature] Improve Hybrid A* planner (`#1594 <https://github.com/CPFL/Autoware/issues/1594>`_)
  * Delete obstacle_sim from astar_planner package, replaced to lidar_fake_perception
  * Modify package name, astar_planner -> waypoint_planner, and create astar_planner library package
  * Delete obstacle_avoid/astar* and modify its dependency to astar_planner library
  * Fix astar_navi with astar_planner library
  * Refactor astar_navi by separating HAstar library and fixing coodinate system
  * Rename obstacle_avoid -> astar_avoid and under refactoring
  * Fix cost function and configures
  * Fix backward search and refactor configurations
  * Apply clang-format
  * Refactor include
  * Fix typo and so on
  * Improve astar_avoid by incremental goal search
  * Apply clang-format
  * Revert package names
  * Fix package/code names
  * Update runtime_manager
  * Improve astar_avoid to execute avoidance behavior by state transition (by rebuild decision maker)
  * Fix PascalCase message names by `#1408 <https://github.com/CPFL/Autoware/issues/1408>`_
  * Remove obstacle_avoid directory
  * Fix default parameter for costmap topic
  * Fix warning and initialize condition
  * Remove use_avoidance_state mode (TODO: after merging rebuild decision maker)
  * Improve astar_avoid behavior by simple state transition and multi-threading
  * Apply clang-format
  * Fix replan_interval in astar_avoid
  * Add descriptions for paramters
  * Rename pkg name, astar_planner -> waypoint_planner
  * Fix param name
  * Fix avoid waypoints height
  * Fix parameter and formalize code
  * Add README for freespace/waypoint_planner
  * Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Fix CHANGELOG
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  Add License terms
  Co-Authored-By: aohsato <aohsato@gmail.com>
  * Fix astar_navi/README.md
  * Add License terms
  * Fix const pointer
  * Added unit test base
  * Restructured folders
  * Fix bug by adding AstarSearch reset
  * Fix WaveFrontNode initialization
  Co-Authored-By: aohsato <aohsato@gmail.com>
  * Fix variable name
  * Refactor threading
  * Re-adding lidar_fake_perception
  * Fix the condition to judge reaching goal
  * Add 'use_decision state' mode to transit avoidance state by decision_maker
  * Fix calcDiffOfRadian (if diff > 2pi)
  * Feature/test astar planner (`#1753 <https://github.com/CPFL/Autoware/issues/1753>`_)
  * Restructured folders
  * Added unit test base
  * Removed remaining folder
  * Test WIP
  * Added astar_util tests and base file for astar_search tests
  * Updated to ROS Cpp Style guidelines
  * Added test for SimpleNode constructor
  * Updated Copyright date
  * Added tests for astar algorithm
  * Added default constructor to WaveFront struct
  * Revert use_state_decision mode (94af7b6)
  * Fix costmap topic names by merging costmap_generator
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
* Contributors: Abraham Monrroy Cano, Akihito Ohsato, amc-nu

1.10.0 (2019-01-17)
-------------------
* Fixes for catkin_make
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
* Contributors: Esteve Fernandez, amc-nu

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
* Contributors: Kosuke Murakami

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
* Contributors: Yamato ANDO

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
* Contributors: Yusuke FUJII

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
* Run visualization node when astar_navi is launched
* Update interface.yaml
* Rewrite astar_navi
* Fix codes to use map_file messages and old vector_map_info topics
* Add module graph tool
* Correct waypoint_follower dependnecy about message header
* Delete old API
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Initial commit for public release
* Contributors: Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, syouji
