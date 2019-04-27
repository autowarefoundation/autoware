^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waypoint_maker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [Feature] waypoint replanner and extractor (`#1951 <https://github.com/CPFL/Autoware/issues/1951>`_)
* Fix license notice in corresponding package.xml
* Cleanup WaypointState.msg and add sttering_state=STR_BACK (`#1822 <https://github.com/CPFL/Autoware/issues/1822>`_)
* Contributors: Abraham Monrroy Cano, Akihito Ohsato, Kenji Funaoka, Yuma Nihei, amc-nu

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
* Fix flashing marker & set lifetime
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* Update README
* Fix date of lisence & add {}
* Return disable_decision_maker to rosparam
* Clang-format
* Split resample method
* Split the velocity limit method.
* Move calculate method of velocity param to init_config
* Optimize accel_decel_limitation_method
* Optimize radius-calculate-method
* Rename waypoint_replan to velocity_replanner
* Fix curve_radius_method on resampling_mode
* Rename cpp and header_files filter->replan
* Rename class and functions filter->replan
* Fix claculate radius function
* Fix CalcVelParam func & add comment
* Modify visualized topic, /base_waypoints -> /lane_waypoints_array, mainly for editing waypoints
* Fix to deleting marker before re-publishing
* Fix bug, increasing path size for each loop
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
* Add end point offset option
* retry Clang-format
* Revert "Clang-format"
  This reverts commit fb284523027c0df1ec513b09c6426c9d235c9f12.
* Ignore space character of waypoints.csv
* Reflect waypoints height updown
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
* Clang-format
* Correspond to new version of waypoint_csv(for decision_maker)
* fix runtime_manager layout and description
* Add config_callback for online waypoint tuning
* Add velocity plan offset for system delay
* fix launch files for ros parameter
* Add waypoint_filter functions
* Fixing `#1064 <https://github.com/CPFL/Autoware/pull/1064>`_ and `#1065 <https://github.com/CPFL/Autoware/pull/1065>`_
* Add deleting marker/buffers for replaying rosbag, and some fix
* Apply clang-format
* Fix typo
* Modify plotting of controller response by metric interval
* Add some rosparam and fix launch file
* Add velocity visualize as facing text
* Fix circular buffer initialization
* Refactor using boost::circular_buffer, ApproximateTimeSyncPolicy, and so on
* Separate speed vizualizer -> waypoint_marker/waypoint_velocity_vizualizer
* Add current_velocity and twist_command line graph
* Remove unused comments
* Add velocity visualizer with 3D graph.
* Contributors: AMC, Abraham Monrroy, Akihito Ohsato, Kosuke Murakami, Yuma, Yuma Nihei

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
* add smoothing on waypoint loader
* add waypoint relay
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
* Add obstacle avoid feature in waypoint_planner
* convert to autoware_msgs
* Contributors: TomohitoAndo, YamatoAndo

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
* Publish local waypoint velocity
* Update interface.yaml for each packages
* Update README.md for waypoint_maker
* Add the function, velocity plannning, for format ver2 and 3
* initial commit for README.md for each packages
* Fix not using reserved word in C++
* Comment out conflict part in visualization, Add Local Point Marker
* Apply clang-format
* extract processing as function
* Rename function
* Add enum class "ChangeFlag"
* Rewrite waypoint_loader
* Add visualization for change flag
* Adjust for new fileformat
* Add checkFileFormat() function
* Add g\_ prefix to global variables
* Add support for multi lane files
* Add no name namespame instead of using static modifier
* Contributors: TomohitoAndo, h_ohta

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Run visualization node when astar_navi is launched
* Publish marker when the traffic light detection is unknown
* Fix codes to use map_file messages and old vector_map_info topics
* Change message type for current velocity , Vector3stamepd -> TwistStamped
* Use clang-format
* Accomodate to vel_pose_mux
* Add module graph tool
* Remove needless compiling flags
* Divide waypoints marker into global and local
* Fix code style
* Delete static modifier,Add noname namespace
* Switch signal detection source by Runtime Manager configuration
* Avoid segmentation fault when parsing waypoint file
* Create verifyFileConsistency function
* Fix some place
* Fix Node name
* Parse old CSV format
* Compute yaw in lane_navi and waypoint_clicker
* Add debug code ,checking the orientation of waypoint
* Delete needless code
* Fix style
* Add Markers which show traffic_waypoints_array
* Rewrite waypoint_clicker by new API
* Change to show LaneArray
* Some Changes
* Load two lanes from csv files
* Change Marker style
* Bug fix
* changed to use yaw in a waypoint
* added yaw in waypoint data
* Make junction more visible
* Show guides for the waypoint_clicker
  The waypoint_clicker have clicked a waypoint freehand so far.
  This commit show guides of waypoint, junction, clicked point and found route.
* Add dependent packages
* modified somethings in computing tab
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* bug fix
* some fix
* published local path marker ,and some fix in order to be easy to see
* published local path marker ,and some fix in order to be easy to see
* changed topic name
* Change subscribing topic from 'safety_waypoint' to 'temporal_waypoints'
* first commit major update for waypoint_saver
* modified velocity_set
* Fix subscribing topic
* Add waypoint_clicker
* Fixed typo
* Add the state lattice motion planning features
* Initial commit for public release
* Contributors: Hiroki Ohta, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, h_ohta, pdsljp, syouji
