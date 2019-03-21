^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package decision_maker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* [Feature] Rebuild decision maker (`#1609 <https://github.com/CPFL/Autoware/issues/1609>`_)
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
* Cleanup WaypointState.msg and add sttering_state=STR_BACK (`#1822 <https://github.com/CPFL/Autoware/issues/1822>`_)
* Contributors: Abraham Monrroy Cano, Kenji Funaoka, amc-nu, s-azumi

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
* Moved configuration messages to autoware_config_msgs
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
* Fix build failures on Aarch64
  Following issues are detected and fixed while building Autoware on Aarch64 host:
  1. FLOAT = abs(FLOAT) doesn't really makes sense. Hence modify that to use std::abs.
  2. Include <numeric> header for using accumulate API.
  3. Include <boost/format.hpp> header for using boost format API's.
  Signed-off-by: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
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
* Suppress compile error
  I encounterd compile error like following:
  ```
  Autoware/ros/src/computing/planning/decision/packages/decision_maker/nodes/decision_maker/decision_maker_node_decision.cpp:86:28: error: ‘accumulate’ is not a member of ‘std’
  ```
  And this can be avoided by this modification.
* fix a merging miss
* fix a typo
* parametrize detection area
* code formatting
* fix stopstate to track update of stopline
* remove a unused line
* add to out for stopline_state
* VelocitySet support to stop by stopline
* fix a out of range
* R.I.P
* change to apply ratio to stopline target
* add ratio for stopline target
* increase spinner thread, and apply clang-format
* Add a transition to stopstate to re-start only manually
* Add new feature that use object detection for stopline state
* R.I.P.
* fix a duplicate state update
* stop light_color publish every state update
* fix a judge intersect stopline
* add new state for lanechange
* add new state for all stopline pause
* fix a build bug
* fix a build bug
* fix a state num name
* fix a state num name
* add support to manual fixing light
* fix a lane_change loop
* change a exploring way for lamp
* fix a importing vectormap
* Contributors: Abraham Monrroy, Dejan Pangercic, Kosuke Murakami, Manato Hirabayashi, Manivannan Sadhasivam, Yusuke FUJII

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
* R.I.P.
* fix a duplicate state update
* stop light_color publish every state update
* add new state for lanechange
* fix a judge intersect stopline
* add new state for all stopline pause
* fix a build bug
* fix a build bug
* fix a state num name
* fix a state num name
* add support to manual fixing light
* fix a moving state
* fix a variable declaration was not enough.
* Add author tag
* Checked coding by cppcheck and apply clang-format
* Add new state
  - TrafficLight State (it will be planning to change "behavior" to
  another category)
  - Crawl(slow speed)
* Add to support multiple lane shift
* Remove debug code and apply clang format
* Fixed:
  - callback
  - laneshift
  Added:
  - publisher for laneid
  - new lanechange flag
  - new param for decisionMaker
* cosme
* add shifted lanes
* add simple obstacle avoid based shifting lane
* Fix not working changed callback
* :put_litter_in_its_place:
* add to insert shift lane
* delete build warning, and change stopline
* update state and remove detection state
* fix a build error
* fix a judge left/right
* Fix a state changing bug
* add support to stopline
* Add feature of to find stopline. and following minor fixes
  - to change vectormap operation to vectormap lib.
  - to change state operation
* refactor lamp control
* apply clang-format
* Support to lanechange similar to state_machine(old) package
* Add support to switch on/off directional indicator
* remove unnecessary code from decisionmaker
* add support to waypoint velocity control by state
* add mps2kmph
* update decisionmaker and related library
  - add multiplelane path recognition
  - renamed euc
* Changed path state recognition to the way based on /lane_waypoints_array
* improve a judge algorithms for right/left-turn in intersection
* Add to support manually decision
* Contributors: Yamato ANDO, Yusuke FUJII

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
* Add build option
* apply clang-format
* update decision maker config
* Add to support dynamical parameter for decision_maker
* Add build flags
* fixed launch file for planner selector
* Fix a build error for decision maker node
* Move the decision part of the state machine library to decision_Maker node. This is WIP.
* fix a header dependencies and remove unnecessary part
* Change file composition
* Add publisher for target velocities
* Removed dynamic reconfigure
* Fixed forgetting to rename(state_machine node to decision_maker node)
* integrate planner_selector package to decision_maker package
* Add decision packages into runtime_manager
* apply clang-format
* organize package files and directories
* Add a decision_maker package
  The decision_maker package determines the intention of what actions the
  local planner and control node should take based on perception nodes,
  global planner nodes, map data, sensor data.
  This commit corresponds only to the following functions.
  - Behavior state recognition
  - Dynamic selection of local planner (It is necessary to change the topic name of local planner)
* Contributors: Yusuke FUJII

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
