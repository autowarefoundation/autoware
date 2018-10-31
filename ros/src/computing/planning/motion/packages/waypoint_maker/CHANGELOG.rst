^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waypoint_maker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add obstacle avoid feature in astar_planner
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
