^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dp_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.0 (2019-01-17)
-------------------
* [fix] SSD detector, cmake colcon (`#1837 <https://github.com/kfunaoka/Autoware/issues/1837>`_)
  * Fixes for new colcon script on ssd cuda based node
  * Fixed to RTM and darknet launch files
  * catkin_fix
  * * catkin & colcon build successfully
  * reverted back run to devel space (for the time being)
* Switch to Apache 2 license (develop branch) (`#1741 <https://github.com/kfunaoka/Autoware/issues/1741>`_)
  * Switch to Apache 2
  * Replace BSD-3 license header with Apache 2 and reassign copyright to the
  Autoware Foundation.
  * Update license on Python files
  * Update copyright years
  * Add #ifndef/define _POINTS_IMAGE_H\_
  * Updated license comment
* Use colcon as the build tool (`#1704 <https://github.com/kfunaoka/Autoware/issues/1704>`_)
  * Switch to colcon as the build tool instead of catkin
  * Added cmake-target
  * Added note about the second colcon call
  * Added warning about catkin* scripts being deprecated
  * Fix COLCON_OPTS
  * Added install targets
  * Update Docker image tags
  * Message packages fixes
  * Fix missing dependency
* Fix Ros/ROS naming convention
* Contributors: Abraham Monrroy Cano, Esteve Fernandez

1.9.1 (2018-11-06)
------------------

1.9.0 (2018-10-31)
------------------
* [fix] PascalCase messages (`#1408 <https://github.com/CPFL/Autoware/issues/1408>`_)
  * Switch message files to pascal case
  * Switch message names to pascal case in Runtime Manager
  * Switch message names to pascal case in *.yaml
  * Rename brake_cmd and steer_cmd to BrakeCmd and SteerCmd in main.yaml
* Contributors: Esteve Fernandez

1.8.0 (2018-08-31)
------------------
* Support old behavior of insert static object for obstacle avoidance testing
  Only one simulated car available in the runtime manager
  update for copywrite note
  insert autoware_build_flags to new nodes
* Add README files for OpenPlanner packages
* Test Simulated Vehicles
  Fix Simulated Vehicle Initialization
  Test Following
  Test Obstacle Avoidance
  Add Visualization information to rviz config file open_planner.rviz
* Modify Map loading for OpenPlanner, now it reads from Autoware vector map messages, old behavior still works but from launch file only.
  Delete way_planner, dp_planner from UI, but they still accessible from roslaunch.
* Fix Vector Map parser problem, tested with three different maps
  Fix Global Planning function for the new map modification
  Add OpenPlanner Simulator for perception, traffic lights, cars
  Add OpenPlanner new version to replace wp_planner and dp_planner
  Remove unnecessary files from OpenPlanner libraries
  Test Global and Local planning
  Test Tracking node (kf_contour_track)
  Test Simulation Nodes
  Test Utility Nodes
* Contributors: Hatem Darweesh, hatem-darweesh

1.5.1 (2017-09-25)
------------------
* fix build error for older indigo version
* Contributors: Yusuke FUJII

1.5.0 (2017-09-21)
------------------
* Add changing topic name option for the planner selector.
* compilation issues
* Contributors: Dejan Pangercic, Yusuke FUJII

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
* Change OpenPlanner stand alone libraries names.
* fix a math library compatibility
* Restructure OpenPlanner for merging autoware develop branch
* Contributors: Yusuke FUJII, hatem-darweesh

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
