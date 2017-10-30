^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waypoint_maker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Resolved merge conflict by new feature
* Add obstacle avoid feature in astar_planner
* convert to autoware_msgs
* Contributors: TomohitoAndo, YamatoAndo, Yusuke FUJII

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
* Merge branch 'master' into develop-waypoint-maker
  Conflicts:
  ros/src/computing/planning/motion/packages/waypoint_follower/lib/libwaypoint_follower.cpp
  ros/src/computing/planning/motion/packages/waypoint_maker/nodes/waypoint_marker_publisher/waypoint_marker_publisher.cpp
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
