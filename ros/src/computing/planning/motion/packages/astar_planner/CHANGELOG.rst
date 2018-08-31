^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package astar_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Esteve Fernandez, Kenji Funaoka

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* Add 'EControl::STOPLINE' state and detected object type 'EObstacleType' to change speed planning
* Separate configration for speed planning against obstacle/stopline (Note: no logics changed)
* Fix checking input for detection in velocity_set::pointsDetection
* VelocitySet support to stop by stopline
* Contributors: Akihito Ohsato, Kosuke Murakami, Yusuke FUJII

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
* Update velocity_set.launch
  set value to false
* fix video settings
* launch file missing parameter setting.
  missing parameter setting, when launch motion plan, report error :enablePlannerDynamicSwitch arg to be set.
  see :  https://github.com/CPFL/Autoware/issues/871
* Contributors: Yamato ANDO, asimay, hironari.yashiro

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
* add web ui
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
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix build issues due to autoware_msgs
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
