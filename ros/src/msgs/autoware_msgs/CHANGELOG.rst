^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-12-11)
------------------
* Merge branch develop into feature/ndt_pcl_gpu
* Merge pull request `#953 <https://github.com/CPFL/Autoware/issues/953>`_ from CPFL/feature/tlr_turn_support
  Feature/tlr turn support, multi traffic signal support
* Added support to publish result of multiple traffic signals according to the lane
  VectorMapServer Support to publish signals on current lane if current_pose and final_waypoints available
* Merge branch 'develop' into feature/tlr_turn
* Initial modifications to feat_proj, tlr, context and vector_map loader, server and client to support different types of traffic signals
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge pull request `#936 <https://github.com/CPFL/Autoware/issues/936>`_ from CPFL/feature/decision
  Feature/decision: Enhancement decision maker node
* Merge branch 'feature/OpenPlanner' into develop
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* Fixed:
  - callback
  - laneshift
  Added:
  - publisher for laneid
  - new lanechange flag
  - new param for decisionMaker
* add to insert shift lane
* merge develop and fixed slow switching map
* Merge branch 'feature/decision' of github.com:cpfl/autoware into feature/decision
* Support to lanechange similar to state_machine(old) package
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Changed path state recognition to the way based on /lane_waypoints_array
* Merge branch 'develop' into feature/ndt_pcl_gpu
  Conflicts:
  ros/src/computing/perception/localization/packages/ndt_localizer/CMakeLists.txt
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_mapping/ndt_mapping.cpp
  ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching/ndt_matching.cpp
* Fix build error, add msg definition
* Merge pull request `#844 <https://github.com/CPFL/Autoware/issues/844>`_ from CPFL/feature/remote_monitor
  Feature/remote monitor
* Rename and merge msgs
* add path velocity smoothing
* add msg of waypointstate for decision_maker
* Merge branch 'master' into feature/decision
* Feature/fusion_filter - fusion multiple lidar (`#842 <https://github.com/CPFL/Autoware/issues/842>`_)
  * Add fusion_filter to merge multiple lidar pointclouds
  * Refactor fusion_filter
  * Apply clang-format and rebase develop
  * Add fusion_filter launch and runtime_manager config
  * Fix names, fusion_filter -> points_concat_filter
  * Fix build error in ros-indigo
  * Fix some default message/frame names
  * Refactor code and apply clang-format
  * Add configrations for runtime_manager
  * Fix CMake
* refactor code
* refactor code
* Merge branch 'master' of github.com:cpfl/autoware into develop
* refactor msg and add blinker to msg
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* Merge pull request `#826 <https://github.com/CPFL/Autoware/issues/826>`_ from CPFL/feature/ray_ground_filter
  Feature/ray ground filter
* Merge branch 'develop' into feature/ray_ground_filter
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/sensing.yaml
* Add ground_filter config for runtime_manager (`#828 <https://github.com/CPFL/Autoware/issues/828>`_)
* Ray Ground Filter Initial Commit
* merge develop
* add approximate_ndt_mapping (`#811 <https://github.com/CPFL/Autoware/issues/811>`_)
* add new msg and rename msg
* add mqtt sender
* Merge branch 'feature/decision_maker' of github.com:cpfl/autoware into feature/remote_monitor
* Contributors: AMC, Akihito Ohsato, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, anhnv-3991, hatem-darweesh, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* update decision maker config
* Add to support dynamical parameter for decision_maker
* Add decision_maker config
* add config parameter
* autoware_msgs does not depend on jsk_rviz_plugin, cmake and package.xml were not correct
* Contributors: Dejan Pangercic, Yusuke FUJII

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* Resolved merge conflict by new feature
* convert to autoware_msgs
* Contributors: YamatoAndo, Yusuke FUJII

1.2.0 (2017-06-07)
------------------

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
