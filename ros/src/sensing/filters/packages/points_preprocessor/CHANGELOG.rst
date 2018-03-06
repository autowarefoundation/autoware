^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package points_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* decoupled ray ground filter into lib and exe, added unit test (`#932 <https://github.com/cpfl/autoware/issues/932>`_)
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* Feature/ring_ground_filter parameter description (`#884 <https://github.com/cpfl/autoware/issues/884>`_)
  * Added a README file for ground_filter tuning
  * Moved and formatted Patipon instructions on ring_ground_filter
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
* Feature/cloud transformer (`#860 <https://github.com/cpfl/autoware/issues/860>`_)
  * Added Cloud transformer node
  transforms pointcloud to a target frame
  * Added support for XYZIR point type
  * Added error checks when transformation unavailable
* Solved conflicts by ring filter config message naming change
* Add ground_filter config for runtime_manager (`#828 <https://github.com/cpfl/autoware/issues/828>`_)
* Added Compilation fix for Kinect
* Added descriptions to the params in launch file
* Ray Ground Filter Initial Commit
* Contributors: AMC, Abraham Monrroy, Akihito Ohsato, Yamato ANDO, christopherho-ApexAI, hatem-darweesh

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
* [hotfix] fixes to lidar_tracker package(`#787 <https://github.com/cpfl/autoware/issues/787>`_)
  -Fixed a typo in the ground_filter launch file from points_preprocessor
  -Fixed ID duplication in kf_lidar_tracker
  Tested on Ubuntu 14.04 and 16.04
* Contributors: Abraham Monrroy, Yusuke FUJII

1.4.0 (2017-08-04)
------------------
* link to documentation
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Update ground_filter.launch
* Update ground_filter.launch
  Added params to launch file
* Typo Fix
* Fixed a bug that caused missing points
* Fixed linking error on 16.04
* Modified as suggested by @dejanpan on `#655 <https://github.com/cpfl/autoware/issues/655>`_
* -Standarized code
  -Added support for the 3 Velodyne Sensors models (use model_sensor {16,32,64})
  -Parametrized
* Test adding interface
* Try to add interface
* New version of ground_filter
* Contributors: AMC, Abraham Monrroy, Dejan Pangercic, Patiphon Narksri

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

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
* Rename package name.
  data_filter -> filters
  points_filter -> points_downsample
* Contributors: yukikitsukawa
