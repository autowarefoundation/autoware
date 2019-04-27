^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waypoint_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Akihito Ohsato

1.10.0 (2019-01-17)
-------------------
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
* Contributors: Esteve Fernandez

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
* Add obstacle avoid feature in waypoint_planner
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
