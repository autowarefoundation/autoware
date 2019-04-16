^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waypoint_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* [Feature] Rebuild decision maker (`#1609 <https://github.com/CPFL/Autoware/issues/1609>`_)
* Revert "Fix/health checker (`#2012 <https://github.com/CPFL/Autoware/issues/2012>`_)" (`#2037 <https://github.com/CPFL/Autoware/issues/2037>`_)
  This reverts commit e4187a7138eb90ad6f119eb35f824b16465aefda.
  Reverts `#2012 <https://github.com/CPFL/Autoware/issues/2012>`_
  Merged without adequate description of the bug or fixes made
* Fix/health checker (`#2012 <https://github.com/CPFL/Autoware/issues/2012>`_)
* Feature/autoware health checker (`#1943 <https://github.com/CPFL/Autoware/issues/1943>`_)
* Fix license notice in corresponding package.xml
* [Fix] negative max_v values for negative omega (`#1880 <https://github.com/CPFL/Autoware/issues/1880>`_)
* Contributors: Geoffrey Biggs, Masaya Kataoka, Sneha Ganesh, amc-nu, s-azumi

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
* Apply clang-format and unity coding rule
* Add sim_lidar frame to wf_simulator
* Contributors: Akihito Ohsato, Esteve Fernandez, amc-nu

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
* fix to get wheel_base
* Add to publish position deviation from waypoint
  - apply clang-format
  - add publisher of position deviation
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Esteve Fernandez, Kenji Funaoka, Yusuke FUJII

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
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
* Added parentheses and fixed the layout
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
* Fix velocity planning at end of waypoint
* add publish function of stop state
* delete unnecessary line
* add ctrl_cmd/cmd/linear_acceletion
* Contributors: Abraham Monrroy, Kosuke Murakami, Yamato ANDO, Yuki Iida, Yuma

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
* Fix build error
* Rename and merge msgs
* refactor code
* refactor code
* refactor code and add ctrl mode topic
* refactor msg and add blinker to msg
* fix bug and refactor code
* add twist gate node
* Contributors: Akihito Ohsato, Yamato ANDO, Yuki Iida

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
* launch files
* added install targets
  some dependencies are not used
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
* hotfix build error due to dependency.
* fix circular-dependency
* Contributors: Shohei Fujii, Yusuke FUJII

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Add param bar of twist filter node in runtime manager
* New simulator with angle and position errors
* Bug fix for linear interpolation flag and command velocity
* Add low pass filter to twist
* Delete unused functions
* Change variable type, extract ros code from PurePursuit Class
* Fix indent
* Move non-ROS initializer outside InitForROS()
* Update CMakeLists.txt
* Add topic publisher for steering robot
* Add new message to control steering robot
* Update comments
* Comment out unused function
* Delete unused value
* Rewrite for applying new template
* Add subscription for closest waypoint
* Adjust for new lane_select
* Adjust for new fileformat
* Add change_flag variable
* Contributors: Takahiro Miki, Yukihiro Saito, h_ohta

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Improve visualization of circular arc
* Change filtering target, angular velocity to linear velocity
* Define const value using e
* Use noname namespace instead of static modifier
* Stop to output debug message at console
* Fix bug of the calculation of the lookahead_distance
* Add constant for minimum curvature and maximum radius of curvature, Fix calcCurvature function
* Change variable name in ConfigWaypointFollower, calculate function for lookahead distance
* Extract pure pursuit algorithm part as Class ,and visualization for ROS
* Add fail safe
* Define vehicle acceleration
* Improve visualization of circular arc
* Change filtering target, angular velocity to linear velocity
* Define const value using e
* Use noname namespace instead of static modifier
* Stop to output debug message at console
* Fix bug of the calculation of the lookahead_distance
* Add constant for minimum curvature and maximum radius of curvature, Fix calcCurvature function
* Change variable name in ConfigWaypointFollower, calculate function for lookahead distance
* Extract pure pursuit algorithm part as Class ,and visualization for ROS
* Add fail safe
* Define vehicle acceleration
* Delete launch command for old model publisher
* Change message type for current velocity , Vector3stamepd -> TwistStamped
* Update interface.yaml in waypoint_follower
* Add module graph tool
* Remove needless compiling flags
* Delete typo
* Use clang-format
* use ax + by + c = 0 as linear equation instead of y = mx + n
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Some fix
* Format code by using clang-format
* Change subscribe topic name
* Fix some parts
* Add choice function for subscribe topic
* Add static modifier
* Delete needless part
* Use unnamed namespace instead of static modifier
* Extract two function from duplicate part ,Change to select next target from next waypoint if next waypoint is first or last
* Delete needless things
* Fix subscribe name
* Delete static modifier , Use unnamed namespace instead
* Change node name from odom_gen to wf_simulator
* Change to set Initial Pose from TF, if initial source is localizer or gnss
* Publish /sim_pose instead of /odom_pose
* Add some error handling codes
* Some fix
* Fix indent
* Fix name of global variable
* Comment out debug code
* Correct vehicle_socket dependnecy about message header
* Correct runtime manager dependencies
* temporary commit
* Add linear interpolate mode Switch
* Bug fix about 'calcTwist'
* Add function , 'verify whether vehicle is following correctly or not'
* Refactoring and Delete needless parts
* Extract as function
* Refactoring
* Added 'getWaypointPose' function into 'WayPoints' class
* Support ZMP CAN
* Use functions in tf instead of self made functions
* Delete needless code
* Fix Style
* Extract the part making odometry and Make the function
* Change launch file name
* Fix Style ,Delete needless code
* Fix to calculate relative angle
* Bug fix for the probrem about range of circle
* Define new msgs in CMakelists
* Create new msgs
* Make getClosestWaypoint() more safety
* Create new Member Function of WayPoints
* Add the function which gets waypoint orientation, Beta Version
* Some fix
* Add default value
* add dependencies
* added lack things
* created ConfigTwistFilter message
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* angular velocity filtering by using lateral acceleration
* changed to use yaw in a waypoint
* minor fix
* bug fix
* prevented segment fault
* fix style
* added comments
* moved definitions into libwaypoint_follower.cpp
* extracted the function which gets linear equation and moved into library
* added some comments
* moved two functions into libwaypoint_follower
* deleted OpenMP settings
* fix typo
* made more stable
* deleted unused class
* minor fix
* fixed trajectory circle visualizer
* cleaned up unused code
* bug fix , deleted unused code
* make more brief
  Conflicts:
  ros/src/computing/planning/motion/packages/waypoint_follower/lib/libwaypoint_follower.cpp
* deleted unused code
  R
* comment outed temporarily
* Refactoring CMakeLists.txt
  Remove absolute paths by using cmake features and pkg-config.
* fix style
* parameterized
* renamed ConfigLaneFollower.msg to ConfigWaypointFollower.msg
* bug fix for model publisher
* modified somethings in computing tab
* bug fix , changed current pose to center of rear tires
* bug fix , changed current pose to center of rear tires
* bug fix for interpolate of waypoint
* comment out fitness evaluation
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Add sleep
* to make more stable
* bug fix for global path
* changed in order not to select shorter target than previous target
* Add new parameters
* Minor fix
* fix in order to adjust argument
* some fix for pure pursuit
* deleted and uncommented unused things
* some fix
* bug fix for current velocity
* fix style
* bug fix and added #ifdef for debug code
* Modify to deal with acceleration
* added averaging filter
* adjusted to velocity_set
* fixed odom_gen
* Change velocity_set.cpp to subscribe 'config/velocity_set'
* Add new variables for DPM detection
* fix style
* Move velocity_set from waypoint_follower to driving_planner
* improved
* deleted unused
* bug fix
* added twist filter node
* deleted collision avoid and twist through
* Add closest_waypoint publisher
* Change private to protected for class inheritance in velocity_set.cpp
* Remove needless function
* adjusted to 'WayPoints' Class and deleted unused code
* added log
* improved
* added new member function , fix SetPath function
* created new class 'Waypoints' and 'Path' class became deprecated
* fix typo
* moved somefunctions from pure pursuit to libwaypoint_follower
* deleted unused code
* erased redundancy
* Change variable name
* first commit for major update of pure pursuit
* Clean code.
* Modified and cleaned code.
* Modify code to avoid sudden aceleration or deceleration.
* added sleep
* modified velocity_set
* modified velocity_set.cpp
* modified velocity_set
* Add the state lattice motion planning features
* Initial commit for public release
* Contributors: Hiroki Ohta, Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, h_ohta, pdsljp
