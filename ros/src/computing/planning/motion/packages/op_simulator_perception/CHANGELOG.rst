^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package op_simulator_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'feature/OpenPlanner' into develop
* merge develop and fixed slow switching map
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
* merge develop
* Contributors: Yamato ANDO, Yuki Iida, Yusuke FUJII, hatem-darweesh, yukikitsukawa

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
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------
* Change OpenPlanner stand alone libraries names.
* Restructure OpenPlanner for merging autoware develop branch
* Add OpenPlanner to Develop Branch, add OpenPlanner to Runtime Manager, and modify rviz default config file
  fix map loading options
  automatic replanning simulation and traffic light stop and go
  add performance logging
  behavior state for traffic light and stop signs fixed
  fix logging shift, fix euclidean clusters problem
  visualize dp steps
  detection config for robot vel16
  tune ff path follower for simulation
  tune ff path follower for simulation
  HMI update
  simulated obstacle bounding box representation
  HMI Update
  HMI Successful Demo
  improve detection accuracy to < 10 cm
  HMI Tested. More runtime manager options.
  HMI Tested. More runtime manager options.
  fix dp plan build issue
  Controller - Steering Delay auto calibration
  Multi-Traffic Behavior Simulation on Rviz using OpenPlanner
  change node names to match ROS naming standards
  change node names to match ROS naming standards
  - Add OpenPlanner Vehicle Simulator
  - Integrate with Autoware's pure pursut
  - Revised local planning
  - Unit-Test usig playback based simulation
  update simulation launch files
  More Unit Testing
  Improve Object Tracking
  CAN info message handle!
  rviz config
  visualization changes
  add option to select velocities source
  RS Planner Test
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
