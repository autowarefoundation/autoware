^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lane_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/tlr_turn
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge pull request `#936 <https://github.com/CPFL/Autoware/issues/936>`_ from CPFL/feature/decision
  Feature/decision: Enhancement decision maker node
* Fixed:
  - callback
  - laneshift
  Added:
  - publisher for laneid
  - new lanechange flag
  - new param for decisionMaker
* apply clang-format
* merge develop and fixed slow switching map
* fix a segv bug when currentpose was changed a lot
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* Support to lanechange similar to state_machine(old) package
* add path velocity smoothing
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Merge branch 'feature/decision_maker' of github.com:cpfl/autoware into feature/remote_monitor
* Contributors: AMC, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* Add decision packages into runtime_manager
* Contributors: Yusuke FUJII

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
* fix circular-dependency
* Contributors: Shohei Fujii

1.1.2 (2017-02-27 23:10)
------------------------
* output log
* publish closest waypoint as -1 when cannot find each closest waypoint in each lane
* Contributors: h_ohta

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Fix style
* Fix hermite curve
* Remove unused
* Fix style
* Change transparency of lane for change
* Change buffer size
* every time find neighbor lanes
* Update README for lane_planner
* parameter from runtime manager
* Fix definition of function
* Change copy to move
* add error avoidance
* optimize for RVO
* Remove unused variable
* Create new lane for lane change
* Add hermite curve library
* Remove debug code
* Fix bug for searching closest wapoint
* Subscribe state
* Publish change flag as topic, which keeps the same value until lane change is finished
* right and left lane index is -1 when closest waypoint on each lane is -1
* Update README.md
* Update README.md
* Apply clang-format
* Refactoring code
* Update visualization
* Add ROS_WARN
* Update interface.yaml for each packages
* Update README.md for lane_planner
* Initialize the closest waypoint number when the vehicle is outside of a search distance
* Delete useless braces
* initial commit for README.md for each packages
* Fix Indent
* Sort definitions
* Rewrite visualizer
* Rewrite lane change processing adding state and new tuple
* Add ROS_INFO about current change_flag
* Move ROS_INFO
* Modify value which will be added into tuple
* Add ChangeFlag value into tuple
* Remove unused code
* Add state variable
* Add include
* Change output to log
* Edit Comment out and WARN message
* Change processing order, Fix not getting neighbor lanes when current lane index is fixed
* Fix keeping storing lane array infinitely in vector
* Fix comment
* Add lane initialization when subscribed lane array
* Rewrite to change local planning to global planning
* Create run function
* bring together initializer for ROS
* Fix include guard
* Delete comment out
* Add launch file for lane_select, fix to use ros parameter
* apply clang-format
* Rewrite lane_select node and add new function
* Contributors: Hiroki Ohta, h_ohta

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Create red and green lanes from waypoint_saver CSV
* Fix codes to use map_file messages and old vector_map_info topics
* Add module graph tool
* Publish cached waypoint
  If configure lane_rule, publish cached waypoint.
* Fix lane_select bug.
  /traffic_waypoints_array よりも先に /config/lane_select が来ると、
  g_lane_array.lanes が空で落ちるため、チェックを追加。
* Switch signal detection source by Runtime Manager configuration
* Correct runtime manager dependencies
* Improve handling junction lane
* Create lane_navi.launch
* Compute yaw in lane_navi and waypoint_clicker
* Change subscribe topic
* Rename topics of LaneArray message
* Delete old API
* Rewrite lane_stop by new API
* Rewrite lane_rule by new API
* Rewrite lane_navi by new API
* Add new API for multiple lanes
* Change two lanes in lane_select
* Merge branch 'master' into develop-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/waypoint_follower/CMakeLists.txt
* Add number_of_zeros_behind parameter
* Rename number_of_zeros parameter
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Merge branch 'master' of https://github.com/CPFL/Autoware into develop-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/driving_planner/nodes/velocity_set/velocity_set.cpp
  ros/src/util/packages/runtime_manager/msg/ConfigVelocitySet.msg
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Make any pramaters configurable
* Support direction angle
* Move error variable declaration
* Add utility for direction angle
* Merge branch 'master' into develop-planner
  Conflicts:
  ros/src/computing/planning/motion/packages/driving_planner/launch/velocity_set.launch
  ros/src/computing/planning/motion/packages/driving_planner/nodes/velocity_set/velocity_set.cpp
* Fix velocity computation on crossroads
* changed topic name
* Fix subscribing topic names
* Cache current waypoints
* Publish without change in default of vmap
* Smooth acceleration and deceleration at crossroads
* Initial commit for public release
* Contributors: Hiroki Ohta, Shinpei Kato, Syohei YOSHIDA, TomohitoAndo, USUDA Hisashi, syouji
