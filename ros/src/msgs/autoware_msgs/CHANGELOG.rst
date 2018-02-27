^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Added support to publish result of multiple traffic signals according to the lane
  VectorMapServer Support to publish signals on current lane if current_pose and final_waypoints available
* Initial modifications to feat_proj, tlr, context and vector_map loader, server and client to support different types of traffic signals
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
* Support to lanechange similar to state_machine(old) package
* Changed path state recognition to the way based on /lane_waypoints_array
* Fix build error, add msg definition
* Rename and merge msgs
* add path velocity smoothing
* add msg of waypointstate for decision_maker
* Feature/fusion_filter - fusion multiple lidar (`#842 <https://github.com/cpfl/autoware/issues/842>`_)
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
* refactor msg and add blinker to msg
* Add ground_filter config for runtime_manager (`#828 <https://github.com/cpfl/autoware/issues/828>`_)
* Ray Ground Filter Initial Commit
* add approximate_ndt_mapping (`#811 <https://github.com/cpfl/autoware/issues/811>`_)
* add new msg and rename msg
* add mqtt sender
* Contributors: AMC, Akihito Ohsato, Yamato ANDO, Yuki Iida, Yuki Kitsukawa, Yusuke FUJII, hatem-darweesh

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
* update decision maker config
* Add to support dynamical parameter for decision_maker
* Add decision_maker config
* add config parameter
* autoware_msgs does not depend on jsk_rviz_plugin, cmake and package.xml were not correct
* Contributors: Dejan Pangercic, Yusuke FUJII

1.4.0 (2017-08-04)
------------------
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* convert to autoware_msgs
* Contributors: YamatoAndo

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
