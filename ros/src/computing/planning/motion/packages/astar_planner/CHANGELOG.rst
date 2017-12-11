^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package astar_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge pull request `#892 <https://github.com/CPFL/Autoware/issues/892>`_ from asimay/patch-1
  launch file missing parameter setting.
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Update velocity_set.launch
  set value to false
* fix video settings
* launch file missing parameter setting.
  missing parameter setting, when launch motion plan, report error :enablePlannerDynamicSwitch arg to be set.
  see :  https://github.com/CPFL/Autoware/issues/871
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Merge branch 'feature/decision_maker' of github.com:cpfl/autoware into feature/remote_monitor
* Merge branch 'develop' of https://github.com/CPFL/Autoware into feature/remote_monitor
* Contributors: Yamato ANDO, Yuki Iida, Yusuke FUJII, asimay, hatem-darweesh, hironari.yashiro, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* Merge pull request `#808 <https://github.com/cpfl/autoware/issues/808>`_ from CPFL/feature/decision_maker
  [WIP]Feature/decision maker `#807 <https://github.com/cpfl/autoware/issues/807>`_
* add web ui
* Merge branch 'develop' of github.com:CPFL/Autoware into feature/decision_maker
* Resolve conflicts
* Change parameter for obstacle avoid
* Change color of a simulated obstacle
* fix a planner selector
  - lane select got to be able to change topicname for planner selector
* Add changing topic name option for the planner selector.
* fix segv
* Add feature to put simulated obstacles in astar planner
* R.I.P.
* apply clang-format
* add multiple crosswalk detection
* Change the method to decide stop point
* Fix indentation
* Add parameter to ignore points nearby the vehicle
* Contributors: TomohitoAndo, Yusuke FUJII, hironari.yashiro

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
