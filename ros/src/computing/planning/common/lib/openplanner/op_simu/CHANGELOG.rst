^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package op_simu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* - Test Tracking
  - Add missing parameter to lunch file and runtime manager
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* - Update OpenPlanner libraries (plannerh, simuh, utilityh) with the latest modifications
  - Fix inconsistency after library update, make sure old (way_planner, dp_planner) are working fine
  - Create new package (op_local_planner)
  - Create common launch file for local planning params
  - Create new node (op_trajectory_generator)
  - Create launch file for trajectory generation only
  - Test generating trajectories (rollouts) in simulation with way_planner
  - Test generating trajectories with real rosbag data with way_planner
  - Test generating trajectories with real rosbag data and waypoints_loader
* Contributors: Yamato ANDO, hatem-darweesh

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

1.2.0 (2017-06-07)
------------------
* Change OpenPlanner stand alone libraries names.
* Contributors: hatem-darweesh

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
