^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dp_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'feature/OpenPlanner' into develop
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* merge develop and fixed slow switching map
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* - Update OpenPlanner libraries (plannerh, simuh, utilityh) with the latest modifications
  - Fix inconsistency after library update, make sure old (way_planner, dp_planner) are working fine
  - Create new package (op_local_planner)
  - Create common launch file for local planning params
  - Create new node (op_trajectory_generator)
  - Create launch file for trajectory generation only
  - Test generating trajectories (rollouts) in simulation with way_planner
  - Test generating trajectories with real rosbag data with way_planner
  - Test generating trajectories with real rosbag data and waypoints_loader
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* Merge pull request `#826 <https://github.com/CPFL/Autoware/issues/826>`_ from CPFL/feature/ray_ground_filter
  Feature/ray ground filter
* Ray Ground Filter Initial Commit
* merge develop
* Merge branch 'feature/decision_maker' of github.com:cpfl/autoware into feature/remote_monitor
* Contributors: AMC, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, hatem-darweesh, yukikitsukawa

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
