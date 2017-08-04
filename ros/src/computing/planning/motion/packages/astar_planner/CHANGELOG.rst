^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package astar_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix build issues due to autoware_msgs
* Resolved merge conflict by new feature
* Apply clang-formt
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
* Fix incorrect check for the waypoint index
* Contributors: TomohitoAndo

1.1.0 (2017-02-24)
------------------
* Use enum class instead of enum
* improve deceleratiion for obstacles
* Make function names more concise
* Decide the number of zero velocity from the position of the obstacle
* Avoid sudden acceleration after changing waypoints for deceleration
* Remove unnecessary calcalation
* Add get size method for new waypoints
* Fix typo
* improve acceleration
* Use integer size with temporal waypoints
* Avoid sudden aceleration after changing waypoints
* Remove unnecessary comments
* Remove unnecessary include
* Merge branch 'develop' of https://github.com/CPFL/Autoware into feature/refactor_velocity_set
  Conflicts:
  ros/src/computing/planning/motion/packages/astar_planner/nodes/velocity_set/velocity_set.cpp
* Remove unnecessary comment
* Comment out publishing of the obstacle marker
* Make constans all capitals
* Make the function more concise
* Use local variables for publishers
* Implement callbacks in class
* Use local variables instead of global variables
* Remove the dependency of libvelocity_set
* Use constexpr for constant values
* Make obstacle detection function more concise
* Modify variable names
* Remove ignore range
* Don't use call by reference with primitive data types
* Remove unused variables
* Fix dependencies
* Remove unused function
* Format comments
* Split class into separate files
* Subscribe closest waypoint
* Contributors: TomohitoAndo

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Ignore close points
* Stop publishing obstacle marker
* Use the result of lidar_tracker
* Change launch file to output log
* Fix license
* Remove needless dependencies
* Remove comments
* Separate motion planning package
* Contributors: TomohitoAndo, pdsljp
