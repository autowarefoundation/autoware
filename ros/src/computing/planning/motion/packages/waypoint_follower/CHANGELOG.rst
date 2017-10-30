^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waypoint_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* launch files
* added install targets
  some dependencies are not used
* Contributors: Dejan Pangercic

1.4.0 (2017-08-04)
------------------

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
* Merge branch 'pp-support-can' into fix-motion-planner
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
* Merge pull request `#99 <https://github.com/CPFL/Autoware/issues/99>`_ from CPFL/driving-planner
  Update driving_planner and computing.yaml
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
* Merge branch 'master' into develop-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/waypoint_follower/CMakeLists.txt
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
* Merge branch 'develop-planner' of https://github.com/CPFL/Autoware into develop-planner
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
* Merge branch 'develop-planner' of https://github.com/CPFL/Autoware into develop-planner
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
* Contributors: Hiroki Ohta, Matthew O'Kelly, Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, h_ohta, pdsljp
